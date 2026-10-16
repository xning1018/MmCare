# -*- coding: utf-8 -*-
import os
import sys
import subprocess

try:
    import docx
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "python-docx"])

from docx import Document
from docx.shared import Pt, RGBColor
from docx.oxml.ns import qn
from docx.enum.text import WD_PARAGRAPH_ALIGNMENT
from docx.enum.table import WD_TABLE_ALIGNMENT


def set_run_font(run, font_name="微软雅黑", size_pt=10.5, bold=False, color_rgb=None):
    run.font.name = font_name
    run._element.rPr.rFonts.set(qn("w:eastAsia"), font_name)
    run.font.size = Pt(size_pt)
    run.bold = bold
    if color_rgb:
        run.font.color.rgb = RGBColor(*color_rgb)


def add_heading_custom(doc, text, level=1):
    h = doc.add_heading("", level=level)
    run = h.add_run(text)
    sizes = {0: 19, 1: 14.5, 2: 12.5, 3: 11}
    colors = {
        0: (13, 148, 136),
        1: (15, 118, 110),
        2: (17, 94, 89),
        3: (30, 41, 59),
    }
    set_run_font(run, "微软雅黑", sizes.get(level, 12), bold=True, color_rgb=colors.get(level, (0, 0, 0)))
    return h


def add_para(doc, text="", bold_prefix="", bullet=False, color_rgb=None):
    p = doc.add_paragraph(style="List Bullet" if bullet else None)
    p.paragraph_format.space_after = Pt(4)
    p.paragraph_format.line_spacing = 1.35
    if bold_prefix:
        r_pre = p.add_run(bold_prefix)
        set_run_font(r_pre, "微软雅黑", 10.5, bold=True, color_rgb=color_rgb or (15, 23, 42))
    if text:
        r_txt = p.add_run(text)
        set_run_font(r_txt, "微软雅黑", 10.5, bold=False, color_rgb=color_rgb or (51, 65, 85))
    return p


def add_styled_table(doc, headers, rows):
    table = doc.add_table(rows=1, cols=len(headers))
    table.style = "Table Grid"
    table.alignment = WD_TABLE_ALIGNMENT.CENTER

    hdr_cells = table.rows[0].cells
    for i, h_text in enumerate(headers):
        hdr_cells[i].text = ""
        p = hdr_cells[i].paragraphs[0]
        r = p.add_run(h_text)
        set_run_font(r, "微软雅黑", 10, bold=True, color_rgb=(15, 118, 110))

    for row_data in rows:
        row_cells = table.add_row().cells
        for i, val in enumerate(row_data):
            row_cells[i].text = ""
            p = row_cells[i].paragraphs[0]
            r = p.add_run(val)
            set_run_font(r, "微软雅黑", 9.5, bold=(i == 0), color_rgb=(30, 41, 59))
    doc.add_paragraph()


def build_special_manual():
    doc = Document()
    doc.styles["Normal"].font.name = "微软雅黑"
    doc.styles["Normal"]._element.rPr.rFonts.set(qn("w:eastAsia"), "微软雅黑")
    doc.styles["Normal"].font.size = Pt(10.5)

    # 标题
    t = add_heading_custom(doc, "MMCare 端侧 AI 真机数据采集、训练与姿态/跌倒判决专题手册", level=0)
    t.alignment = WD_PARAGRAPH_ALIGNMENT.CENTER
    sub = doc.add_paragraph()
    sub.alignment = WD_PARAGRAPH_ALIGNMENT.CENTER
    r_sub = sub.add_run("为什么这么做 · 采集全流程手把手操作 · 主板姿态与跌倒计算原理 · 避坑注意事项")
    set_run_font(r_sub, "微软雅黑", 11, bold=True, color_rgb=(100, 116, 139))

    # ==================== 专题一 ====================
    add_heading_custom(doc, "专题一：为什么要做真机数据采集？采集的数据是怎么变成主板判断结果的？", level=1)

    add_heading_custom(doc, "1.1 为什么要采集这 4 类真机动作数据？", level=2)
    add_para(
        doc,
        "传统毫米波雷达仅靠固定的速度或高度阈值判断跌倒，极易出现两大痛点：① 人走快一点、突然转身或者弯腰捡东西就误报跌倒；② 换了一个房间或换了雷达安装挂高后参数失效。为此，本系统在瑞萨 RA6M5 主板上内置了 PINN-CfC（物理内嵌液态神经网络），专门将人体动作划分为 4 个核心类别进行训练与识别：",
    )
    add_para(doc, "类别 0（Class 0 - 正常慢走）：让主板认识日常散步、慢走的平稳波形。", bold_prefix="• ", bullet=True)
    add_para(doc, "类别 1（Class 1 - 快走 & 急转身）：这是最容易触发传统雷达误报的动作（速度大、加速度大，但人并没有倒地静止）。专门采集它作为负样本，教会主板把“快走/急转身”与“真摔倒”严格区分开！", bold_prefix="• ", bullet=True)
    add_para(doc, "类别 2（Class 2 - 弯腰 & 下蹲）：这也是最容易误报的日常动作（有垂直下蹲速度，高度下降，但下蹲过程可控且无失衡冲击）。专门采集它作为负样本，教会主板不要把弯腰捡东西或蹲下当成摔倒！", bold_prefix="• ", bullet=True)
    add_para(doc, "类别 3（Class 3 - 失衡摔倒）：正样本，特征是“前半段急速失衡下坠 + 冲击度 Jerk 极大 + 后半段躺地静止不动”。", bold_prefix="• ", bullet=True)

    add_heading_custom(doc, "1.2 从“采集数据”到“主板计算上传大屏”的完整闭环流程", level=2)
    add_para(doc, "您在雷达前做动作 → 开发板通过 4G 实时上传距离与速度流 → 电脑采集软件录下轨迹并提取 16 维物理特征存入 datasets/real_posture_data.npz。", bold_prefix="第 1 步（真机采样）：")
    add_para(doc, "点击软件上的“一键训练”按钮 → 电脑将您采集的真机样本与物理运动学基座样本混合，训练 PINN-CfC 液态神经网络 → 进行对称 INT8 量化 → 自动生成并覆盖工程里的 C 语言权重头文件 API/ai_weights.h。", bold_prefix="第 2 步（训练导出 C 权重）：")
    add_para(doc, "在 Keil MDK 中按 F7 编译、F8 烧录 → 新的 API/ai_weights.h 被烧录进瑞萨 RA6M5 单片机 Flash 中。上电时单片机自动将其反量化为 32 位浮点数（FP32）放入 512KB 高速 SRAM。", bold_prefix="第 3 步（烧录进主板）：")
    add_para(doc, "主板每 100ms 采集一次 LD2450（24GHz）与 LD6002C（60GHz）的双雷达数据，提取 16 维特征送入神经网络，实时算出当前的姿态分类 posture_class（Class 0/1/2/3）与跌倒触发码（111/1/11），并通过 4G 上传给云服务器，最终在 Web 大屏上实时显示出【站立 / 正常 / 正常慢走 / 快走 & 急转身 / 弯腰 & 下蹲 / 摔倒】！", bold_prefix="第 4 步（主板实时推理与大屏显示）：")

    # ==================== 专题二 ====================
    add_heading_custom(doc, "专题二：如何采集真机数据？手把手分步操作指南", level=1)

    add_heading_custom(doc, "第一步：开机热机并打开图形化采集中心", level=2)
    add_para(doc, "将 RA6M5 开发板上电（无需额外接 USB 转串口线，采集软件直接通过 4G 云端拉取开发板实时数据）。", bold_prefix="1. ")
    add_para(doc, "等待开机前 5 秒冷启动保护期结束，在雷达前正常走两步。", bold_prefix="2. ")
    add_para(doc, "在电脑上打开工程目录（c:\\Users\\ls\\RASmartConfigurator\\Mmcare），双击运行【启动AI姿态采集与训练.bat】（或【双击启动_AI采集与训练界面.bat】）。", bold_prefix="3. ")
    add_para(doc, "弹出“MMCare 瑞萨 RA6M5 · 分步倒计时引导式 AI 姿态采集与训练中心”窗口后，看窗口右上角状态栏：显示“📡 板端4G: 在线 | 实时距离: X.XXm | 速度: X.Xcm/s”即代表连接成功！", bold_prefix="4. ")

    add_heading_custom(doc, "第二步：依次点击底部 4 个按钮录制真机动作（自带 3 秒准备倒计时）", level=2)
    add_para(
        doc,
        "为什么会有 3 秒倒计时？因为您点完鼠标后需要时间从电脑桌走到雷达前面的空地上！所以每次点击按钮后，电脑都会先倒数 3、2、1（伴随三声短促的“滴-滴-滴”提示音）。当您听到一声长音“滴——”时，才正式开始录制：",
    )

    add_styled_table(
        doc,
        ["按钮名称", "录制时长", "听到长音“滴——”后的标准动作做法", "建议录制次数", "对应的主板 AI 类别"],
        [
            [
                "① 采集【正常慢走】\n(青色按钮)",
                "25 秒 / 次",
                "在雷达正前方 1m ~ 4m 范围内，像平时散步一样自然来回慢走，直到听到两声“滴-滴”结束音。",
                "点击录制 1~2 次\n(自动切分出多组样本)",
                "Class 0：正常慢走\n(主板上传码 21)",
            ],
            [
                "② 采集【快走 & 急转身】\n(蓝色按钮)",
                "25 秒 / 次",
                "在雷达前快速大步走过、走到一半突然急停并转身往回走、大幅度摆臂，持续做 25 秒。",
                "点击录制 1~2 次",
                "Class 1：快走 & 急转身\n(主板上传码 22，防误报)",
            ],
            [
                "③ 采集【弯腰 & 下蹲】\n(橙色按钮)",
                "25 秒 / 次",
                "站在雷达前 1.5m~3m 处，反复做“原地弯腰捡东西 -> 站起 -> 慢速蹲下停顿 2 秒 -> 站起”动作。",
                "点击录制 1~2 次",
                "Class 2：弯腰 & 下蹲\n(主板上传码 23，防误报)",
            ],
            [
                "④ 录制 1 次【真实摔倒】\n(红色按钮)",
                "6 秒 / 次",
                "站在软垫旁，听到长音后立即快速倒在垫子上（或快速深蹲到底），然后全身保持一动不动静止 3~4 秒，直到听到两声结束音再起身！",
                "重复点击录制 5~10 次\n(可换不同位置/朝向)",
                "Class 3：摔倒\n(主板上传码 111 / 1 / 11)",
            ],
        ],
    )

    add_para(doc, "中途取消与清空功能：", bold_prefix="💡 ")
    add_para(doc, "如果某次点完按钮后您没站好位置，随时点击窗口中间的【⏹️ 停止当前倒计时】按钮，该次录制会立即作废，绝不会保存错误数据；如果想把以前录的样本全部清空重新录，点击【🗑️ 清空真机样本】即可。", bullet=True)

    add_heading_custom(doc, "第三步：一键训练生成权重并烧录进主板", level=2)
    add_para(doc, "动作录制完成后，点击窗口最下方的绿色大按钮：【🚀 一键训练 PINN-CfC 液态神经网络 并自动更新 C 权重头文件 (API/ai_weights.h)】。", bold_prefix="1. ")
    add_para(doc, "等待约 3~5 秒，窗口中央大字会显示：“🎉 INT8量化后独立验证集准确率 XX.XX% | 跌倒召回率 XX% | 误报率 XX%”，说明新的权重已经自动写入了工程的 API/ai_weights.h 文件中！", bold_prefix="2. ")
    add_para(doc, "打开 Keil MDK（Mmcare.uvprojx），点击 Build（F7）编译工程，再点击 Download（F8）烧录到 RA6M5 开发板中，主板就会立刻使用您刚刚训练好的新模型进行实时姿态与跌倒计算！", bold_prefix="3. ")

    # ==================== 专题三 ====================
    add_heading_custom(doc, "专题三：主板是怎么计算 6 种姿态与跌倒数据的？（底层数据原理详解）", level=1)

    add_heading_custom(doc, "3.1 主板实时提取的 16 维双雷达融合物理数据", level=2)
    add_para(
        doc,
        "无论是 LD2450（通道 A，24GHz 轨迹与多普勒雷达）还是 LD6002C（通道 B，60GHz 三维点云雷达），主板在每 100ms 的推理步中都会在 16 个采样点（约 1.6 秒滑动窗口）内提取 16 个核心物理量送入 PINN-CfC 液态神经网络：",
    )
    add_para(doc, "空间位置与高度坍塌特征（feat[0..1]）：平均径向距离 mean_r、最大位移跨度 span_r（通道 B 额外叠加 LD6002C 三维点云的相对垂直高度落差 rel_y_shift，专门抓人体往地面倒下时的高度坍塌）。", bullet=True)
    add_para(doc, "速度与前后半窗对比特征（feat[2..5]）：窗口平均速度 mean_v、瞬时峰值速度 max_v（通道 B 融合了 50% 三维速度 + 30% 垂直下坠速度 -dH/dt + 20% 径向速度）、前半窗速度 mean_v_early、后半窗速度 mean_v_late。", bullet=True)
    add_para(doc, "冲击动力学特征（feat[6..8]）：峰值加速度 max_acc、峰值加加速度（冲击度 max_jerk）、速度过零率 zero_crossings。", bullet=True)
    add_para(doc, "物理守恒核心特征（feat[9..11]）：速度骤降坍塌比 collapse_ratio（前半段快、后半段慢的比值）、跌后静止度 post_stillness（倒地后位置标准差越小越接近 1.0）、物理耦合冲击指数 pinn_impact（通道 B 额外乘上三维点云卧姿宽高比增益 rel_aspect_gain，因为人躺地时点云横宽竖扁）。", bullet=True)
    add_para(doc, "64 点微多普勒频谱特征（feat[12..15]）：对窗口速度序列做汉宁窗 + 64 点零填充 FFT，提取低频（躯干主运动）到高频（四肢剧烈甩动）4 个频段的能量占比。", bullet=True)

    add_heading_custom(doc, "3.2 主板上传给 Web 大屏的姿态码与跌倒触发码对照表", level=2)
    add_para(doc, "主板（src/hal_entry.c）每帧根据端侧 AI 引擎（API/ai_fall_engine.c）的计算结果，将状态码打包进 4G JSON 报文发往云端，Web 大屏（dashboard.html）收到后直接渲染显示：")

    add_styled_table(
        doc,
        ["主板上传状态码", "Web 大屏显示状态", "主板端侧计算与触发条件（API/ai_fall_engine.c + src/hal_entry.c）"],
        [
            [
                "触发码 111",
                "摔倒\n[触发代码: 111]",
                "由通道 A（LD2450 24GHz）独立触发：开机满 5 秒且 16 点滑窗填满后，神经网络跌倒概率 probs[3] ≥ 56% 且 峰值速度 ≥ 1.05m/s 且 跌后静止度 ≥ 0.40；或强冲击直通达标。",
            ],
            [
                "触发码 1",
                "摔倒\n[触发代码: 1]",
                "由通道 B（LD6002C 60GHz）独立触发：开机满 5 秒后，3D 点云神经网络跌倒概率 probs_60[3] ≥ 56% 且 峰值三维下坠合成速度 ≥ 0.95m/s 且 跌后静止度 ≥ 0.42 且 坍塌比 ≥ 0.25；或底层 0x0E02 跌倒帧置位。",
            ],
            [
                "触发码 11",
                "摔倒\n[触发代码: 11]",
                "LD2450 + LD6002C 双雷达同时共振触发：通道 A 与通道 B 在同一时刻均判定为跌倒（最高置信度）。",
            ],
            [
                "状态码 22",
                "快走 & 急转身",
                "未跌倒时，主板 PINN-CfC 液态神经网络输出 posture_class == Class 1 (AI_POSTURE_FAST_TURN)。",
            ],
            [
                "状态码 23",
                "弯腰 & 下蹲",
                "未跌倒时，主板 PINN-CfC 液态神经网络输出 posture_class == Class 2 (AI_POSTURE_BEND_SQUAT)。",
            ],
            [
                "状态码 21",
                "正常慢走",
                "未跌倒时，主板 PINN-CfC 液态神经网络输出 posture_class == Class 0 (AI_POSTURE_SLOW_WALK) 且存在移动速度 (|V| ≥ 18 cm/s)。",
            ],
            [
                "状态码 24",
                "正常\n(正常站立/洗浴)",
                "未跌倒时，有人在场且处于原地轻微日常活动/洗浴状态 (5 ≤ |V| < 18 cm/s)。",
            ],
            [
                "状态码 20",
                "站立",
                "未跌倒时，有人在场且保持原地静止站立状态 (|V| < 5 cm/s)。",
            ],
        ],
    )

    # ==================== 专题四 ====================
    add_heading_custom(doc, "专题四：数据采集与日常测试的 8 大核心注意事项（必看避坑）", level=1)

    add_para(doc, "1. 采集或测试“摔倒”时，为什么倒下后必须保持静止 2~3 秒？", bold_prefix="")
    add_para(
        doc,
        "因为真实长者跌倒后无法立即起身，主板 AI 专门通过后半窗口的“跌后静止度（post_stillness）”和“速度坍塌比（collapse_ratio）”来区分“真摔倒”和“路过/弯腰”。如果您倒下后立刻又站起来走动，后半窗不静止，主板就会将其归类为“快走 & 急转身”或“弯腰 & 下蹲”而不报跌倒。因此无论是录制按钮 ④ 还是平时测试跌倒，请遵循【快速倒下/深蹲 + 原地静止不动 2~3 秒】的动作要领！",
        bullet=True,
    )

    add_para(doc, "2. 采集时必须等开发板上电在线后再点按钮：", bold_prefix="")
    add_para(
        doc,
        "采集软件内置了防呆保护：如果开发板没上电、或者倒计时期间雷达前没有任何人动（有效数据点少于 6 个），软件会自动提示“⚠️ 未检测到有效人体运动，本次空数据已自动丢弃”，绝不会把空数据写进训练集。",
        bullet=True,
    )

    add_para(doc, "3. 每次训练完后，一定要在 Keil 里按 F7 编译 + F8 烧录：", bold_prefix="")
    add_para(
        doc,
        "在电脑采集软件上点击“一键训练”后，它更新的是电脑硬盘上的 API/ai_weights.h 源码文件。您必须打开 Keil MDK 按 F7 重新编译并按 F8 下载烧录到 RA6M5 开发板，主板芯片里运行的权重才会真正更新！",
        bullet=True,
    )

    add_para(doc, "4. 开机或按复位键后的前 5 秒不会报警，随后请先正常走两步：", bold_prefix="")
    add_para(
        doc,
        "前 5 秒是冷启动保护期（消除开机粒子收敛误报）。满 5 秒后您在雷达前正常走动 3~5 秒，LD6002C 会自动标定当前挂高下的站立高度基线，无需手动修改安装高度代码。",
        bullet=True,
    )

    add_para(doc, "5. 为什么有时候看到触发代码是 111，有时候是 1 或 11？", bold_prefix="")
    add_para(
        doc,
        "因为 LD2450（24GHz）探测范围最广（0.5~6m），对水平/径向冲击最敏感，在中远距离或侧向跌倒时通常由它率先触发（报 111）；而当您在两个雷达正前方近距离（1~2.5m）快速垂直倒地时，LD6002C 的 3D 点云高度坍塌也会同时触发（报 1 或双雷达共振 11）。三者均为有效硬件报警。",
        bullet=True,
    )

    add_para(doc, "6. 微信与 QQ 邮箱推送有 7 秒防刷屏冷却：", bold_prefix="")
    add_para(
        doc,
        "连续测试跌倒时，两次微信/邮件推送之间需间隔满 7 秒，防止高频请求被推送平台限流。",
        bullet=True,
    )

    add_para(doc, "7. 浏览器只保留一个监控大屏标签页：", bold_prefix="")
    add_para(
        doc,
        "请勿同时打开多个大屏标签页，避免旧网页在后台抢发推送。",
        bullet=True,
    )

    add_para(doc, "8. Keil 32KB 编译体积注意事项：", bold_prefix="")
    add_para(
        doc,
        "当前包含完整双雷达端侧 AI 与姿态直传的固件编译体积为 32,608 Bytes（严格小于 Keil 评估版 32,768 Bytes 上限），0 Error(s), 0 Warning(s)。",
        bullet=True,
    )

    desktop_special = r"C:\Users\ls\Desktop\MMCare_真机数据采集训练与姿态跌倒专题手册.docx"
    project_special = r"c:\Users\ls\RASmartConfigurator\Mmcare\MMCare_真机数据采集训练与姿态跌倒专题手册.docx"
    doc.save(desktop_special)
    doc.save(project_special)
    print("Saved special manual to Desktop:", desktop_special)

    # 同时尝试更新桌面的总手册（如果用户未在 Word 中占用）
    try:
        doc.save(r"C:\Users\ls\Desktop\MMCare_完整工程说明与使用手册.docx")
        print("Also updated MMCare_完整工程说明与使用手册.docx on Desktop")
    except PermissionError:
        print("Note: MMCare_完整工程说明与使用手册.docx is currently open in Word, skipped overwriting it.")


if __name__ == "__main__":
    build_special_manual()
