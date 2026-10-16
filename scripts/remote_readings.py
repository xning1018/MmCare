"""设备上报数据的落库与业务派生。

这一层负责把"一帧读数"翻译成：
  * radar_reading 一行时序数据
  * 浴室会话的开始 / 结束（→ 滞留分钟数、洗澡频次）
  * 跌倒事件 + 黑匣子日志
所有写操作都在一个事务里完成，避免出现"有读数没会话"的中间态。
"""
from __future__ import annotations

import json
from typing import Any

from config import BATHROOM_DWELL_ALERT_MINUTES
from database import WRITE_LOCK, get_conn, now_text, query_one
from schemas import ReadingIn
from services import alerts, virtual_sensors

POSE_TEXT = {
    "away": "无人",
    "standing": "站立",
    "sitting": "坐姿",
    "lying": "躺卧",
    "falling": "疑似跌倒",
}


def _open_session(conn, device_id: str) -> dict[str, Any] | None:
    row = conn.execute(
        "SELECT * FROM bathroom_session WHERE device_id = ? AND exit_ts IS NULL ORDER BY id DESC LIMIT 1",
        (device_id,),
    ).fetchone()
    return dict(row) if row else None


def _close_session(conn, session_id: int, enter_ts: str, ts: str) -> int:
    from database import parse_ts

    start = parse_ts(enter_ts)
    end = parse_ts(ts)
    dwell = int((end - start).total_seconds()) if start and end else 0
    dwell = max(dwell, 0)
    conn.execute(
        "UPDATE bathroom_session SET exit_ts = ?, dwell_seconds = ? WHERE id = ?",
        (ts, dwell, session_id),
    )
    return dwell


def ingest(payload: ReadingIn) -> dict[str, Any]:
    device_id = payload.device_id.strip().upper()
    ts = payload.ts or now_text()
    created = now_text()

    # 浴室在场判定优先级：显式 bathroom_present > person 字段
    if payload.bathroom_present is not None:
        present = bool(payload.bathroom_present)
    elif payload.person is not None:
        present = bool(payload.person)
    else:
        present = None

    # ---- 虚拟补全 ----
    # LD2450 只能给「有人/无人、距离、速度」；温度、湿度、跌倒、姿态它给不了。
    # 这里为**设备没上报的字段**生成连续变化的模拟值，
    # 设备真实上报的字段一律以设备为准。被补的字段名记进 raw 便于追溯。
    filled, virtual_fields = virtual_sensors.fill(
        device_id,
        present=present,
        real_fall=bool(payload.fall),
        has_temperature=payload.temperature is not None,
        has_humidity=payload.humidity is not None,
        has_pose=payload.pose is not None,
        has_fall_confidence=payload.fall_confidence is not None,
    )

    temperature = payload.temperature if payload.temperature is not None else filled.get("temperature")
    humidity = payload.humidity if payload.humidity is not None else filled.get("humidity")
    pose = payload.pose if payload.pose is not None else filled.get("pose")
    is_fall = bool(payload.fall) or bool(filled.get("fall"))
    fall_confidence = (
        payload.fall_confidence
        if payload.fall_confidence is not None
        else filled.get("fall_confidence")
    )
    raw_meta = {}
    if virtual_fields:
        raw_meta['virtual'] = virtual_fields
    if payload.h_m is not None:
        raw_meta['h_m'] = payload.h_m
    if payload.v60_ms is not None:
        raw_meta['v60_ms'] = payload.v60_ms
    if payload.v_drop_ms is not None:
        raw_meta['v_drop_ms'] = payload.v_drop_ms
    raw = json.dumps(raw_meta, ensure_ascii=False) if raw_meta else None

    # 写操作串行化：SQLite 同一时刻只允许一个写事务，
    # 而 execute() 走的也是同一把锁，两者不会互相争抢。
    WRITE_LOCK.acquire()
    conn = None
    try:
        conn = get_conn()
        # 1) 设备档案 + 心跳
        conn.execute(
            """
            INSERT INTO devices (device_id, last_seen, online, created_at)
            VALUES (?, ?, 1, ?)
            ON CONFLICT(device_id) DO UPDATE SET
                last_seen = excluded.last_seen, online = 1, offline_since = NULL
            """,
            (device_id, ts, created),
        )

        # 2) 读数入库
        cur = conn.execute(
            """
            INSERT INTO radar_reading
            (device_id, ts, temperature, humidity, distance, motion_speed, person, pose, raw, created_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                device_id,
                ts,
                temperature,
                humidity,
                payload.distance,
                payload.motion_speed,
                None if present is None else int(present),
                pose,
                raw,
                created,
            ),
        )
        reading_id = cur.lastrowid

        # 3) 浴室会话维护
        session = _open_session(conn, device_id)
        session_event: str | None = None
        dwell_seconds: int | None = None
        overstay = False

        if present is True and session is None:
            conn.execute(
                "INSERT INTO bathroom_session (device_id, enter_ts, created_at) VALUES (?, ?, ?)",
                (device_id, ts, created),
            )
            session_event = "enter"
        elif present is False and session is not None:
            dwell_seconds = _close_session(conn, session["id"], session["enter_ts"], ts)
            session_event = "exit"
            if dwell_seconds > BATHROOM_DWELL_ALERT_MINUTES * 60:
                # ⚠️ 这里绝对不能直接 alerts.log_event()：
                # 本事务尚未提交，外连接仍持有 SQLite 写锁，而 log_event()
                # 会另开一条连接去写 —— 两边互锁，要等满 busy_timeout(30s) 才报错，
                # 期间所有其它写入（设备上报/巡检/登录）全部超时。
                # 先记标记，等事务提交、连接关闭后再写日志。
                overstay = True

        # 4) 跌倒事件
        # source 区分来源：真实上报记为 radar，服务端补全的记为 virtual，
        # 端侧 AI 模型上线后可以直接按 source 过滤掉模拟跌倒。
        fall_created = False
        if is_fall:
            fall_source = "virtual" if filled.get("fall") else "radar"
            conn.execute(
                """
                INSERT INTO fall_event (device_id, ts, confidence, handled, source, created_at)
                VALUES (?, ?, ?, 0, ?, ?)
                """,
                (device_id, ts, fall_confidence, fall_source, created),
            )
            fall_created = True

        conn.commit()
    finally:
        if conn is not None:
            conn.close()
        WRITE_LOCK.release()

    # 5) 日志（放在事务外，落库成功后再写）
    if fall_created:
        alerts.log_event(
            "critical",
            "fall",
            "🚨 [紧急] 检测到跌倒事件，请立即确认长者状态！",
            device_id=device_id,
            ts=ts,
        )
    if overstay:
        alerts.log_event(
            "critical",
            "bathroom_overstay",
            f"🚨 [紧急] 浴室滞留已超过 {BATHROOM_DWELL_ALERT_MINUTES} 分钟，请立即确认长者状态！",
            device_id=device_id,
            ts=ts,
        )
    if session_event == "enter":
        alerts.log_event("info", "bathroom_enter", "ℹ️ [提示] 长者已进入浴室，开始计时。", device_id=device_id, ts=ts)
    elif session_event == "exit":
        mins = round((dwell_seconds or 0) / 60, 1)
        alerts.log_event(
            "info", "bathroom_exit", f"ℹ️ [提示] 长者已离开浴室，本次滞留 {mins} 分钟。", device_id=device_id, ts=ts
        )

    return {
        "reading_id": reading_id,
        "device_id": device_id,
        "ts": ts,
        "session_event": session_event,
        "dwell_seconds": dwell_seconds,
        "fall": fall_created,
        # 本帧由服务端补全（而非设备上报）的字段，便于前端/调试追溯
        "virtual_fields": virtual_fields,
    }


def latest_reading(device_id: str) -> dict[str, Any] | None:
    return query_one(
        "SELECT * FROM radar_reading WHERE device_id = ? ORDER BY id DESC LIMIT 1",
        (device_id,),
    )


def recent_readings(device_id: str, limit: int = 100) -> list[dict[str, Any]]:
    from database import query_all

    limit = max(1, min(limit, 1000))
    return query_all(
        "SELECT * FROM radar_reading WHERE device_id = ? ORDER BY id DESC LIMIT ?",
        (device_id, limit),
    )


def has_active_fall(device_id: str) -> bool:
    """最近一次事件是否是"仍未处理"的跌倒（用于旧接口的 fall 字段）。"""
    latest_fall = query_one(
        "SELECT id, handled FROM fall_event WHERE device_id = ? ORDER BY id DESC LIMIT 1",
        (device_id,),
    )
    if latest_fall is None:
        return False
    return not latest_fall["handled"]


def bathroom_state(device_id: str) -> dict[str, Any]:
    """浴室状态：优先看当前是否有打开的会话。"""
    from database import query_one as q1, seconds_since

    open_row = q1(
        "SELECT * FROM bathroom_session WHERE device_id = ? AND exit_ts IS NULL ORDER BY id DESC LIMIT 1",
        (device_id,),
    )
    if open_row is not None:
        elapsed = seconds_since(open_row["enter_ts"]) or 0
        return {
            "in_bathroom": True,
            "status_text": "长者在浴室",
            "current_dwell_seconds": int(elapsed),
            "current_dwell_minutes": round(elapsed / 60, 1),
            "enter_ts": open_row["enter_ts"],
            "dwell_threshold_minutes": BATHROOM_DWELL_ALERT_MINUTES,
            "over_threshold": elapsed > BATHROOM_DWELL_ALERT_MINUTES * 60,
        }

    last = q1(
        "SELECT * FROM bathroom_session WHERE device_id = ? AND exit_ts IS NOT NULL ORDER BY id DESC LIMIT 1",
        (device_id,),
    )
    return {
        "in_bathroom": False,
        "status_text": "长者不在浴室",
        "current_dwell_seconds": 0,
        "current_dwell_minutes": 0,
        "last_dwell_seconds": (last or {}).get("dwell_seconds"),
        "last_dwell_minutes": round((last or {}).get("dwell_seconds", 0) / 60, 1) if last else None,
        "last_exit_ts": (last or {}).get("exit_ts"),
        "dwell_threshold_minutes": BATHROOM_DWELL_ALERT_MINUTES,
        "over_threshold": False,
    }


def bathroom_stats(device_id: str, days: int = 7) -> dict[str, Any]:
    """洗澡频次统计：按天聚合会话数。"""
    from database import query_all

    rows = query_all(
        """
        SELECT substr(enter_ts, 1, 10) AS day,
               COUNT(*) AS sessions,
               SUM(COALESCE(dwell_seconds, 0)) AS total_seconds
        FROM bathroom_session
        WHERE device_id = ?
        GROUP BY day
        ORDER BY day DESC
        LIMIT ?
        """,
        (device_id, max(1, min(days, 90))),
    )
    today = now_text()[:10]
    today_sessions = next((r["sessions"] for r in rows if r["day"] == today), 0)
    return {
        "days": [
            {"date": r["day"], "sessions": r["sessions"], "minutes": round((r["total_seconds"] or 0) / 60, 1)}
            for r in rows
        ],
        "today_sessions": today_sessions,
        "total_sessions": sum(r["sessions"] for r in rows),
    }


def fall_events(device_id: str | None = None, limit: int = 50, only_unhandled: bool = False) -> list[dict[str, Any]]:
    from database import query_all

    limit = max(1, min(limit, 500))
    sql = "SELECT * FROM fall_event"
    clauses: list[str] = []
    params: list[Any] = []
    if device_id:
        clauses.append("device_id = ?")
        params.append(device_id)
    if only_unhandled:
        clauses.append("handled = 0")
    if clauses:
        sql += " WHERE " + " AND ".join(clauses)
    sql += " ORDER BY id DESC LIMIT ?"
    params.append(limit)

    events = query_all(sql, tuple(params))
    for e in events:
        e["handled"] = bool(e["handled"])
        e["message"] = "🚨 [紧急] 检测到跌倒，请立即确认长者状态！"
    return events


def handle_fall_event(event_id: int) -> bool:
    from database import execute, query_one as q1

    if q1("SELECT id FROM fall_event WHERE id = ?", (event_id,)) is None:
        return False
    execute("UPDATE fall_event SET handled = 1 WHERE id = ?", (event_id,))
    return True
