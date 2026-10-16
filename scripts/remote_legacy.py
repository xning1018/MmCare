import json
"""兼容层：改造前已调通的旧接口保持可用，内部转为读新表。

旧调用方                 新接口
POST /api/data      ->    POST /api/v1/readings
GET  /api/data/latest  -> GET /api/v1/radar/latest + /api/v1/device/status
GET  /api/data/history -> GET /api/v1/radar/history
"""
from __future__ import annotations

from fastapi import APIRouter, Depends, Query

from database import now_text
from schemas import ReadingIn
from security import require_api_key
from services import monitor, readings

# ⚠️ 这里必须挂鉴权依赖：legacy 路由在 main.py 里独立 include，
# 不会继承 v1 路由的依赖。漏掉的话 /api/data 就是完全裸奔的写接口，
# 内网穿透暴露公网后任何人都能往里灌假数据、伪造跌倒报警。
router = APIRouter(dependencies=[Depends(require_api_key)])


@router.post("/data", summary="【兼容】设备上报")
def legacy_post_data(body: ReadingIn):
    result = readings.ingest(body)
    return {
        "code": 200,
        "message": "数据保存成功",
        "timestamp": result["ts"],
        "data": result,
        "hint": "建议迁移到 POST /api/v1/readings",
    }


@router.get("/data/latest", summary="【兼容】最新数据")
def legacy_latest(device_id: str = Query(default="MM001")):
    device_id = device_id.strip().upper()
    row = readings.latest_reading(device_id)
    if row is None:
        return {"code": 404, "message": "暂无数据"}
    status = monitor.device_status(device_id) or {}
    return {
        "code": 200,
        "data": {
            "device_id": row["device_id"],
            "temperature": row["temperature"],
            "humidity": row["humidity"],
            "distance": row["distance"],
            "motion_speed": row["motion_speed"],
            "person": None if row["person"] is None else bool(row["person"]),
            "pose": row["pose"],
            "fall": readings.has_active_fall(device_id),
            "online": status.get("online"),
            "timestamp": row["ts"],
            "h_m": (json.loads(row["raw"]).get("h_m") if row.get("raw") and row["raw"] else None),
            "v60_ms": (json.loads(row["raw"]).get("v60_ms") if row.get("raw") and row["raw"] else None),
            "v_drop_ms": (json.loads(row["raw"]).get("v_drop_ms") if row.get("raw") and row["raw"] else None),
        },
        "server_time": now_text(),
    }


@router.get("/data/history", summary="【兼容】历史数据")
def legacy_history(device_id: str = Query(default="MM001"), limit: int = Query(default=100, ge=1, le=1000)):
    device_id = device_id.strip().upper()
    rows = readings.recent_readings(device_id, limit)
    data = [
        {
            "device_id": r["device_id"],
            "temperature": r["temperature"],
            "humidity": r["humidity"],
            "distance": r["distance"],
            "motion_speed": r["motion_speed"],
            "person": None if r["person"] is None else bool(r["person"]),
            "pose": r["pose"],
            "timestamp": r["ts"],
        }
        for r in rows
    ]
    return {"code": 200, "count": len(data), "data": data}
