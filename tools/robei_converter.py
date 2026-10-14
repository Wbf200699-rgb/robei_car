#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robei_converter.py
==================
Robei 可视化工具文件格式双向转换器

支持的转换：
  1. Verilog (.v)        → Robei 模块 (.model)
  2. Robei 模块 (.model) → Verilog (.v)
  3. XDC 约束 (.xdc)    → Robei 约束 (.const)
  4. Robei 约束 (.const) → XDC 约束 (.xdc)

用法示例：
  python robei_converter.py v2model   my_module.v
  python robei_converter.py model2v   my_module.model
  python robei_converter.py xdc2const top.xdc --top top
  python robei_converter.py const2xdc top.const

作者：自动生成（由 AI agent 辅助）
"""

import re
import sys
import os
import argparse
import xml.etree.ElementTree as ET
from html import unescape

# ─────────────────────────────────────────────────────────────────────────────
# 颜色池：Robei 对每个端口分配不同颜色
# ─────────────────────────────────────────────────────────────────────────────
PORT_COLORS = [
    "#faebd7", "#00ffff", "#7fffd4", "#f0ffff", "#f5f5dc",
    "#ffe4c4", "#b8860b", "#ffebcd", "#0000ff", "#8a2be2",
    "#a52a2a", "#deb887", "#5f9ea0", "#7fff00", "#d2691e",
    "#ff7f50", "#6495ed", "#fff8dc", "#dc143c", "#00ffff",
    "#00008b", "#008b8b", "#b8860b", "#a9a9a9", "#006400",
    "#bdb76b", "#8b008b", "#556b2f", "#ff8c00", "#9932cc",
    "#8b0000", "#e9967a", "#8fbc8f", "#483d8b", "#2f4f4f",
    "#00ced1", "#9400d3", "#ff1493", "#00bfff", "#696969",
]

def get_color(idx: int) -> str:
    return PORT_COLORS[idx % len(PORT_COLORS)]


# ═════════════════════════════════════════════════════════════════════════════
# 公共工具函数
# ═════════════════════════════════════════════════════════════════════════════

def encode_robei_code(code: str) -> str:
    """将代码编码为 Robei XML 属性值格式（&#xa; 换行 + XML 转义）。"""
    code = code.replace('&', '&amp;')
    code = code.replace('<', '&lt;')
    code = code.replace('>', '&gt;')
    code = code.replace('"', '&quot;')
    code = code.replace('\r\n', '\n').replace('\r', '\n')
    code = code.replace('\n', '&#xa;')
    return code


def decode_robei_code(encoded: str) -> str:
    """将 Robei Code 属性值还原为可读代码。"""
    text = encoded.replace('&#xa;', '\n')
    text = unescape(text)
    return text


def expand_roplace_get_ports_lists(code: str) -> str:
    """Expand XDC ``get_ports {a b}`` lists for Robei's strict parser.

    Vivado accepts a brace-delimited list in one ``get_ports`` expression, but
    the Roplace parser bundled with Robei asserts while parsing that form.  A
    separate command for each port is equivalent for the I/O properties kept
    in a ``.const`` module and is accepted by both tools.
    """
    expanded = []
    pattern = re.compile(
        r'^(?P<prefix>.*?\[get_ports\s+)\{(?P<ports>[^{}]+)\}'
        r'(?P<suffix>\].*)$'
    )
    for line in code.splitlines():
        match = pattern.match(line)
        if not match:
            expanded.append(line)
            continue

        ports = match.group('ports').split()
        if not ports:
            expanded.append(line)
            continue
        for port in ports:
            expanded.append(
                f"{match.group('prefix')}{port}{match.group('suffix')}"
            )
    return '\n'.join(expanded)


def port_y_ratio(idx: int, total: int, margin: float = 0.05) -> float:
    """计算端口在画布边上的相对位置（0~1）。"""
    if total == 0:
        return 0.5
    step = (1.0 - 2 * margin) / max(total, 1)
    return margin + step * idx + step / 2


def fmt_coord(value: float) -> str:
    """按 Robei 常见写法输出坐标，避免固定补零。"""
    return f'{value:.7g}'


def trim_module_body(body_text: str) -> str:
    """去掉模块体外层空行，同时保留有效代码行的原始缩进。"""
    body_text = body_text.replace('\r\n', '\n').replace('\r', '\n')
    body_text = re.sub(r'^(?:[ \t]*\n)+', '', body_text)
    body_text = re.sub(r'(?:\n[ \t]*)+$', '', body_text)
    return body_text


# ═════════════════════════════════════════════════════════════════════════════
# 第一部分：Verilog → .model
# ═════════════════════════════════════════════════════════════════════════════

def parse_verilog_module(v_text: str) -> dict:
    """
    解析 Verilog 源码，提取模块名、端口列表和模块体代码。

    返回 dict：
      name       : str
      ports      : [{'name', 'inout', 'datatype', 'datasize'}, ...]
      body_code  : str  （模块体，去除端口声明行之外的内容）
    """
    # 去掉注释。Robei Code 属性只保存模块体，不依赖源码注释。
    v_text = re.sub(r'//[^\n]*', '', v_text)
    v_text = re.sub(r'/\*.*?\*/', '', v_text, flags=re.DOTALL)

    def find_matching_paren(text: str, open_pos: int) -> int:
        depth = 0
        for pos in range(open_pos, len(text)):
            ch = text[pos]
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    return pos
        raise ValueError("module 端口列表括号不匹配。")

    def parse_width(width_text: str) -> int:
        wm = re.search(r'\[\s*(\d+)\s*:\s*(\d+)\s*\]', width_text)
        if not wm:
            return 1
        return abs(int(wm.group(1)) - int(wm.group(2))) + 1

    def split_names(name_text: str):
        for part in name_text.split(','):
            part = part.strip()
            if not part:
                continue
            part = part.split('=', 1)[0].strip()
            nm = re.match(r'([A-Za-z_]\w*)', part)
            if nm:
                yield nm.group(1)

    # 模块名
    m = re.search(r'\bmodule\s+(\w+)', v_text)
    if not m:
        raise ValueError("找不到 module 关键字，请检查 Verilog 文件。")
    module_name = m.group(1)

    # 解析第一个 module 的参数区、端口区和模块体。
    pos = m.end()
    while pos < len(v_text) and v_text[pos].isspace():
        pos += 1
    if pos < len(v_text) and v_text[pos] == '#':
        pos += 1
        while pos < len(v_text) and v_text[pos].isspace():
            pos += 1
        if pos < len(v_text) and v_text[pos] == '(':
            pos = find_matching_paren(v_text, pos) + 1
        while pos < len(v_text) and v_text[pos].isspace():
            pos += 1
    if pos >= len(v_text) or v_text[pos] != '(':
        raise ValueError("找不到 module 端口列表。")
    port_open = pos
    port_close = find_matching_paren(v_text, port_open)
    header_ports = v_text[port_open + 1:port_close]
    semi = v_text.find(';', port_close)
    if semi < 0:
        raise ValueError("module 头部缺少分号。")
    end_match = re.search(r'\bendmodule\b', v_text[semi + 1:], re.DOTALL)
    end_pos = semi + 1 + end_match.start() if end_match else len(v_text)
    body_code = trim_module_body(v_text[semi + 1:end_pos])

    # ── 解析端口 ──────────────────────────────────────────────────────────
    ports = []
    port_index = {}

    def add_or_update_port(name: str, direction: str, datatype: str, datasize: int):
        if name in ('module', 'endmodule', 'begin', 'end', 'reg', 'wire',
                    'always', 'assign', 'if', 'else'):
            return
        info = {
            'name':     name,
            'inout':    direction or 'input',
            'datatype': datatype or 'wire',
            'datasize': datasize or 1,
        }
        if name in port_index:
            ports[port_index[name]].update(info)
        else:
            port_index[name] = len(ports)
            ports.append(info)

    def parse_decl_rest(direction: str, rest: str):
        rest = rest.strip()
        datatype = 'wire'
        dm = re.match(r'\b(wire|reg)\b\s*(.*)$', rest, re.DOTALL)
        if dm:
            datatype = dm.group(1)
            rest = dm.group(2).strip()
        sm = re.match(r'\bsigned\b\s*(.*)$', rest, re.DOTALL)
        if sm:
            rest = sm.group(1).strip()
        wm = re.match(r'((?:\[[^\]]+\]\s*)*)(.*)$', rest, re.DOTALL)
        width_text = wm.group(1) if wm else ''
        name_text = wm.group(2).strip() if wm else rest
        datasize = parse_width(width_text)
        for pname in split_names(name_text):
            add_or_update_port(pname, direction, datatype, datasize)
        return direction, datatype, datasize

    # ANSI 风格端口：input wire clk, output reg [7:0] data ...
    last_direction = ''
    last_datatype = 'wire'
    last_datasize = 1
    for item in header_ports.split(','):
        item = item.strip()
        if not item:
            continue
        dm = re.match(r'\b(input|output|inout)\b\s*(.*)$', item, re.DOTALL)
        if dm:
            last_direction, last_datatype, last_datasize = parse_decl_rest(
                dm.group(1), dm.group(2)
            )
        else:
            for pname in split_names(item):
                add_or_update_port(
                    pname, last_direction, last_datatype, last_datasize
                )

    # 非 ANSI 风格端口：module m(a,b); input a; output reg b;
    body_for_port_decls = re.sub(
        r'\bfunction\b.*?\bendfunction\b', '', body_code, flags=re.DOTALL
    )
    body_for_port_decls = re.sub(
        r'\btask\b.*?\bendtask\b', '', body_for_port_decls, flags=re.DOTALL
    )
    body_decl_re = re.compile(r'\b(input|output|inout)\b\s+([^;]+);',
                              re.DOTALL)
    for decl in body_decl_re.finditer(body_for_port_decls):
        parse_decl_rest(decl.group(1), decl.group(2))

    return {
        'name':      module_name,
        'ports':     ports,
        'body_code': body_code,
    }


def verilog_to_model(v_text: str,
                     canvas_w: int = 1600,
                     canvas_h: int = 1080) -> str:
    """将 Verilog 文本转换为 Robei .model XML 字符串。"""
    info        = parse_verilog_module(v_text)
    module_name = info['name']
    ports       = info['ports']
    body_code   = info['body_code']

    encoded_code = encode_robei_code(body_code)

    inputs  = [p for p in ports if p['inout'] == 'input']
    outputs = [p for p in ports if p['inout'] == 'output']
    inouts  = [p for p in ports if p['inout'] == 'inout']

    port_w, port_h = 30, 30
    port_xml_list  = []
    color_idx      = 0

    # 左侧：input
    for i, p in enumerate(inputs):
        y = port_y_ratio(i, len(inputs))
        c = get_color(color_idx); color_idx += 1
        x = -port_w / canvas_w
        port_xml_list.append(
            f' <Port Color="{c}" Connect="" Datasize="{p["datasize"]}" '
            f'Datatype="{p["datatype"]}" Description="" Height="{port_h}" '
            f'Inout="input" Name="{p["name"]}" Parent="{module_name}" '
            f'Side="left" Width="{port_w}" X="{fmt_coord(x)}" Y="{fmt_coord(y)}"/>'
        )

    # 右侧：output
    for i, p in enumerate(outputs):
        y = port_y_ratio(i, len(outputs))
        c = get_color(color_idx); color_idx += 1
        x = 1.0 - port_w / canvas_w
        port_xml_list.append(
            f' <Port Color="{c}" Connect="" Datasize="{p["datasize"]}" '
            f'Datatype="{p["datatype"]}" Description="" Height="{port_h}" '
            f'Inout="output" Name="{p["name"]}" Parent="{module_name}" '
            f'Side="right" Width="{port_w}" X="{fmt_coord(x)}" Y="{fmt_coord(y)}"/>'
        )

    # 底侧：inout
    for i, p in enumerate(inouts):
        x = port_y_ratio(i, len(inouts))
        c = get_color(color_idx); color_idx += 1
        y = 1.0 - port_h / canvas_h
        port_xml_list.append(
            f' <Port Color="{c}" Connect="" Datasize="{p["datasize"]}" '
            f'Datatype="{p["datatype"]}" Description="" Height="{port_h}" '
            f'Inout="inout" Name="{p["name"]}" Parent="{module_name}" '
            f'Side="bottom" Width="{port_w}" X="{fmt_coord(x)}" Y="{fmt_coord(y)}"/>'
        )

    ports_xml = '\n'.join(port_xml_list)

    return (
        f'<!DOCTYPE Robei>\n'
        f'<Module Category="Current" Class="module" Code="{encoded_code}" '
        f'Color="#d3d3d3" Comment="" Height="{canvas_h}" Name="{module_name}" '
        f'Parameters="" Parent="0" Type="model" Width="{canvas_w}" X="0" Y="0">\n'
        f'{ports_xml}\n'
        f'</Module>\n'
    )


# ═════════════════════════════════════════════════════════════════════════════
# 第二部分：.model → Verilog
# ═════════════════════════════════════════════════════════════════════════════

def model_to_verilog(model_xml: str) -> str:
    """将 Robei .model XML 字符串转换为 Verilog 源码。

    Robei 的叶子模型把行为代码保存在根 ``Module`` 的 ``Code`` 属性中；
    系统顶层则由直属的 ``Module`` 与 ``Wire`` 元素描述结构。两种模型都
    使用同一个 ``.model`` 后缀，因此这里需要同时支持行为级和结构级导出。
    """
    # ET 不支持 DOCTYPE，先去掉
    xml_clean = re.sub(r'<!DOCTYPE[^>]*>', '', model_xml)
    root = ET.fromstring(xml_clean)

    module_name = root.get('Name', 'unnamed_module')
    raw_code    = root.get('Code', '')
    body_code   = decode_robei_code(raw_code)

    ports = []
    for el in root.findall('Port'):
        pname = el.get('Name', '')
        if not pname:
            continue
        ports.append({
            'name':     pname,
            'inout':    el.get('Inout', 'input'),
            'datatype': el.get('Datatype', 'wire'),
            'datasize': int(el.get('Datasize', '1')),
        })

    # 生成端口声明
    port_decls = []
    for p in ports:
        direction = p['inout']
        dtype     = p['datatype']
        size      = p['datasize']
        name      = p['name']
        width     = f'[{size-1}:0] ' if size > 1 else ''
        if direction == 'output' and dtype == 'reg':
            port_decls.append(f'  output reg {width}{name}')
        else:
            port_decls.append(f'  {direction} {dtype} {width}{name}')

    ports_str = ',\n'.join(port_decls)

    child_modules = root.findall('Module')
    model_wires = root.findall('Wire')

    # 叶子模型没有直属子模块，继续保持原有的 Code -> Verilog 行为。
    if not child_modules:
        return (
            f'module {module_name} (\n'
            f'{ports_str}\n'
            f');\n'
            f'\n'
            f'{body_code}\n'
            f'\n'
            f'endmodule\n'
        )

    if not model_wires:
        raise ValueError(
            f'结构模型 {module_name} 包含子模块，但没有任何 Wire 连线。'
        )

    ident_re = re.compile(r'^[A-Za-z_][A-Za-z0-9_$]*$')

    def require_identifier(value: str, what: str):
        if not ident_re.match(value or ''):
            raise ValueError(f'{what} 不是可直接导出的 Verilog 标识符: {value!r}')

    def parse_endpoint(value: str):
        """把 ``top#instance>port`` 或 ``top>port`` 转成稳定的元组键。"""
        if '>' not in value:
            raise ValueError(f'Robei Wire 端点缺少 ">": {value!r}')
        owner, port_name = value.split('>', 1)
        require_identifier(port_name, '端口名')
        if '#' in owner:
            parent_name, instance_name = owner.split('#', 1)
            if parent_name != module_name:
                raise ValueError(
                    f'Wire 端点父模块不匹配: {value!r}, 期望 {module_name!r}'
                )
            require_identifier(instance_name, '实例名')
            return ('child', instance_name, port_name)
        if owner != module_name:
            raise ValueError(
                f'Wire 顶层端点不匹配: {value!r}, 期望 {module_name!r}'
            )
        return ('root', port_name)

    require_identifier(module_name, '模块名')
    root_port_map = {p['name']: p for p in ports}

    children = []
    child_map = {}
    child_port_map = {}
    for child in child_modules:
        class_name = child.get('Class', '')
        instance_name = child.get('Name', '')
        require_identifier(class_name, '子模块类型')
        require_identifier(instance_name, '子模块实例名')
        if instance_name in child_map:
            raise ValueError(f'结构模型中存在重复实例名: {instance_name}')

        child_ports = []
        for port in child.findall('Port'):
            port_name = port.get('Name', '')
            require_identifier(port_name, f'{instance_name} 的端口名')
            info = {
                'name': port_name,
                'inout': port.get('Inout', 'input'),
                'datatype': port.get('Datatype', 'wire'),
                'datasize': int(port.get('Datasize', '1')),
            }
            if info['inout'] not in ('input', 'output', 'inout'):
                raise ValueError(
                    f'{instance_name}.{port_name} 的方向无效: {info["inout"]}'
                )
            child_ports.append(info)
            child_port_map[(instance_name, port_name)] = info

        info = {
            'class': class_name,
            'name': instance_name,
            'ports': child_ports,
            'parameters': (child.get('Parameters') or '').strip(),
        }
        children.append(info)
        child_map[instance_name] = info

    def endpoint_port_info(endpoint):
        if endpoint[0] == 'root':
            info = root_port_map.get(endpoint[1])
            label = f'{module_name}.{endpoint[1]}'
        else:
            info = child_port_map.get((endpoint[1], endpoint[2]))
            label = f'{endpoint[1]}.{endpoint[2]}'
        if info is None:
            raise ValueError(f'Wire 引用了不存在的端口: {label}')
        return info, label

    wires = []
    incoming = {}
    outgoing = {}
    for wire in model_wires:
        source = parse_endpoint(wire.get('From', ''))
        destination = parse_endpoint(wire.get('To', ''))
        source_info, source_label = endpoint_port_info(source)
        dest_info, dest_label = endpoint_port_info(destination)

        if source_info['inout'] not in ('output', 'inout'):
            if not (source[0] == 'root' and source_info['inout'] == 'input'):
                raise ValueError(f'Wire 驱动端方向错误: {source_label}')
        if dest_info['inout'] not in ('input', 'inout'):
            if not (destination[0] == 'root' and
                    dest_info['inout'] == 'output'):
                raise ValueError(f'Wire 接收端方向错误: {dest_label}')

        wire_size = int(wire.get('Datasize', '1'))
        if wire_size != source_info['datasize']:
            raise ValueError(
                f'Wire {wire.get("Name", "")} 与驱动端 {source_label} 位宽不符: '
                f'{wire_size} != {source_info["datasize"]}'
            )
        if wire_size != dest_info['datasize']:
            raise ValueError(
                f'Wire {wire.get("Name", "")} 与接收端 {dest_label} 位宽不符: '
                f'{wire_size} != {dest_info["datasize"]}'
            )
        if destination in incoming:
            raise ValueError(f'端口被多个 Wire 驱动: {dest_label}')

        wire_name = wire.get('Name', '')
        require_identifier(wire_name, 'Wire 名')
        record = {
            'name': wire_name,
            'size': wire_size,
            'source': source,
            'destination': destination,
        }
        wires.append(record)
        incoming[destination] = record
        outgoing.setdefault(source, []).append(record)

    # 一个子模块输出可能扇出到多个 Wire。若它直接连接顶层输出，就采用
    # 顶层端口作为实际网络；否则采用第一条 Wire 的名字作为内部网络。
    source_expr = {}
    internal_nets = []
    internal_net_names = set()
    for source, source_wires in outgoing.items():
        if source[0] == 'root':
            source_expr[source] = source[1]
            continue

        root_destinations = [
            item['destination'][1]
            for item in source_wires
            if item['destination'][0] == 'root'
        ]
        if root_destinations:
            expression = root_destinations[0]
        else:
            expression = source_wires[0]['name']
            if expression in root_port_map:
                raise ValueError(f'内部网络与顶层端口重名: {expression}')
            if expression not in internal_net_names:
                internal_nets.append((expression, source_wires[0]['size']))
                internal_net_names.add(expression)
        source_expr[source] = expression

    # 根输出若不是被选为子模块输出的实际网络，需要补一个连续赋值。
    output_assigns = []
    for record in wires:
        destination = record['destination']
        if destination[0] != 'root':
            continue
        expression = source_expr[record['source']]
        if expression != destination[1]:
            output_assigns.append(f'  assign {destination[1]} = {expression};')

    net_lines = []
    for name, size in internal_nets:
        width = f'[{size-1}:0] ' if size > 1 else ''
        net_lines.append(f'  wire {width}{name};')

    instance_blocks = []
    for child in children:
        connections = []
        for port in child['ports']:
            endpoint = ('child', child['name'], port['name'])
            if port['inout'] in ('input', 'inout') and endpoint in incoming:
                expression = source_expr[incoming[endpoint]['source']]
            elif port['inout'] in ('output', 'inout') and endpoint in outgoing:
                expression = source_expr[endpoint]
            else:
                expression = ''
            connections.append(f'    .{port["name"]}({expression})')

        params = child['parameters']
        if params:
            raise ValueError(
                f'实例 {child["name"]} 含未支持的 Parameters 属性: {params!r}'
            )
        joined_connections = ',\n'.join(connections)
        instance_blocks.append(
            f'  {child["class"]} {child["name"]} (\n'
            f'{joined_connections}\n'
            f'  );'
        )

    sections = []
    if net_lines:
        sections.append('\n'.join(net_lines))
    if body_code.strip():
        sections.append(body_code.strip())
    if output_assigns:
        sections.append('\n'.join(output_assigns))
    sections.append('\n\n'.join(instance_blocks))
    structural_body = '\n\n'.join(sections)

    return (
        f'module {module_name} (\n'
        f'{ports_str}\n'
        f');\n'
        f'\n'
        f'{structural_body}\n'
        f'\n'
        f'endmodule\n'
    )


# ═════════════════════════════════════════════════════════════════════════════
# 第三部分：XDC → .const
# ═════════════════════════════════════════════════════════════════════════════

def parse_xdc(xdc_text: str):
    """
    解析 XDC 文件，返回：
      pin_map   : { port_name: {pin, iostandard, drive, slew, pull} }
      extra_code: str  （create_clock 等其他命令）
    """
    pin_map     = {}
    extra_lines = []

    def ensure_port(port):
        if port not in pin_map:
            pin_map[port] = {'pin': '', 'iostandard': 'LVCMOS33',
                             'drive': '', 'slew': '', 'pull': 'none'}

    def ports_from_group(raw: str):
        """从 [get_ports {a b c}] 或 [get_ports portname] 中提取端口列表。"""
        raw = raw.strip().strip('{}')
        return [p.strip() for p in raw.split() if p.strip()]

    for line in xdc_text.splitlines():
        s = line.strip()
        if not s or s.startswith('#'):
            extra_lines.append(line)
            continue

        # PACKAGE_PIN
        m = re.match(
            r'set_property\s+PACKAGE_PIN\s+(\S+)\s+\[get_ports\s+\{?(\S+?)\}?\]',
            s, re.IGNORECASE)
        if m:
            pin, port = m.group(1), m.group(2)
            ensure_port(port)
            pin_map[port]['pin'] = pin
            continue

        # IOSTANDARD（支持多端口组）
        m = re.match(
            r'set_property\s+IOSTANDARD\s+(\S+)\s+\[get_ports\s+\{?(.*?)\}?\]',
            s, re.IGNORECASE)
        if m:
            std, raw = m.group(1), m.group(2)
            for port in ports_from_group(raw):
                ensure_port(port)
                pin_map[port]['iostandard'] = std
            continue

        # DRIVE
        m = re.match(
            r'set_property\s+DRIVE\s+(\S+)\s+\[get_ports\s+\{?(.*?)\}?\]',
            s, re.IGNORECASE)
        if m:
            drv, raw = m.group(1), m.group(2)
            for port in ports_from_group(raw):
                ensure_port(port)
                pin_map[port]['drive'] = drv
            continue

        # SLEW
        m = re.match(
            r'set_property\s+SLEW\s+(\S+)\s+\[get_ports\s+\{?(.*?)\}?\]',
            s, re.IGNORECASE)
        if m:
            slew, raw = m.group(1), m.group(2)
            for port in ports_from_group(raw):
                ensure_port(port)
                pin_map[port]['slew'] = slew
            continue

        # PULLUP
        m = re.match(
            r'set_property\s+PULLUP\s+true\s+\[get_ports\s+\{?(.*?)\}?\]',
            s, re.IGNORECASE)
        if m:
            for port in ports_from_group(m.group(1)):
                ensure_port(port)
                pin_map[port]['pull'] = 'up'
            continue

        # PULLDOWN
        m = re.match(
            r'set_property\s+PULLDOWN\s+true\s+\[get_ports\s+\{?(.*?)\}?\]',
            s, re.IGNORECASE)
        if m:
            for port in ports_from_group(m.group(1)):
                ensure_port(port)
                pin_map[port]['pull'] = 'down'
            continue

        # 其他（create_clock, set_clock_groups, CFGBVS 等）
        extra_lines.append(line)

    extra_code = '\n'.join(extra_lines).strip()
    return pin_map, extra_code


def xdc_to_const(xdc_text: str,
                 top_module_name: str = 'top',
                 device: str = 'xc7a50tfgg484',
                 family: str = 'artix7',
                 vendor: str = 'AMD') -> str:
    """
    将 XDC 约束文件转换为 Robei .const XML 格式。

    生成的 .const 包含：
      - 顶层 Module（Type="const"）
      - 每个物理引脚对应一个 Port 元素（Name=PIN号, Voltage=IOSTANDARD...）
      - Code 属性存储 create_clock 等额外命令

    Wire 连接（引脚↔模块端口）需在 Robei GUI 中完成，
    或通过 --model 参数提供对应 .model 文件自动生成（见扩展功能）。
    """
    pin_map, extra_code = parse_xdc(xdc_text)
    encoded_extra = encode_robei_code(extra_code)

    ports_list = sorted(pin_map.items())   # [(signal_name, props), ...]
    total      = len(ports_list)

    # 按左右平均分配
    left_ports  = ports_list[:total // 2 + total % 2]
    right_ports = ports_list[total // 2 + total % 2:]

    # 动态调整画布高度
    n_rows   = max(len(left_ports), len(right_ports), 1)
    canvas_h = max(1080, n_rows * 55 + 200)
    canvas_w = max(1600, 2000)

    port_w, port_h = 30, 30
    port_xml_list  = []
    color_idx      = 0

    for i, (sig, props) in enumerate(left_ports):
        y = port_y_ratio(i, len(left_ports))
        c = get_color(color_idx); color_idx += 1
        x = -port_w / canvas_w
        port_xml_list.append(
            f'  <Port Color="{c}" Description="" Drive="{props["drive"]}" '
            f'Height="{port_h}" Name="{props["pin"] or sig}" '
            f'Parent="{top_module_name}" Pull="{props["pull"]}" '
            f'Slew="{props["slew"]}" '
            f'Side="left" Voltage="{props["iostandard"]}" '
            f'Width="{port_w}" X="{x:.7f}" Y="{y:.7f}"/>'
        )

    for i, (sig, props) in enumerate(right_ports):
        y = port_y_ratio(i, len(right_ports))
        c = get_color(color_idx); color_idx += 1
        x = 1.0 - port_w / canvas_w
        port_xml_list.append(
            f'  <Port Color="{c}" Description="" Drive="{props["drive"]}" '
            f'Height="{port_h}" Name="{props["pin"] or sig}" '
            f'Parent="{top_module_name}" Pull="{props["pull"]}" '
            f'Slew="{props["slew"]}" '
            f'Side="right" Voltage="{props["iostandard"]}" '
            f'Width="{port_w}" X="{x:.7f}" Y="{y:.7f}"/>'
        )

    ports_xml = '\n'.join(port_xml_list)

    return (
        f'<!DOCTYPE Robei>\n'
        f'<Module Class="module" Code="{encoded_extra}" Color="#d3d3d3" '
        f'Device="{device}" Family="{family}" Height="{canvas_h}" '
        f'Name="{top_module_name}" Parent="0" Type="const" '
        f'Vendor="{vendor}" Width="{canvas_w}" X="0" Y="0">\n'
        f'{ports_xml}\n'
        f'</Module>\n'
    )


# ═════════════════════════════════════════════════════════════════════════════
# 第四部分：.const → XDC
# ═════════════════════════════════════════════════════════════════════════════

def const_to_xdc(const_xml: str) -> str:
    """
    将 Robei .const XML 转换为 XDC 约束文件。

    .const 中 Port 的 Name 是物理引脚号，Wire 元素记录了
    "top>PIN → top#instance>portname" 的映射，从而还原信号名。
    """
    xml_clean = re.sub(r'<!DOCTYPE[^>]*>', '', const_xml)
    root = ET.fromstring(xml_clean)

    # 建立 PIN -> signal_name 映射。Robei 会根据连线操作把物理端口
    # 放在 From 或 To 任意一侧，因此这里不能假定 Wire 的固定方向。
    pin_to_signal = {}
    for wire in root.findall('Wire'):
        pin_name = None
        sig_name = None
        for endpoint in (wire.get('From', ''), wire.get('To', '')):
            if '>' not in endpoint:
                continue
            owner, port_name = endpoint.split('>', 1)
            if owner == root.get('Name', 'top'):
                pin_name = port_name
            elif owner.startswith(root.get('Name', 'top') + '#'):
                sig_name = port_name
        if pin_name and sig_name:
            pin_to_signal[pin_name] = sig_name

    # 收集顶层 Port（仅直属 root 的 Port，跳过子 Module 内的 Port）
    ports_data = []
    for port_el in root.findall('Port'):
        pin  = port_el.get('Name', '')
        vol  = port_el.get('Voltage', 'LVCMOS33')
        drv  = port_el.get('Drive', '')
        slew = port_el.get('Slew', '')
        pull = port_el.get('Pull', 'none').lower()
        sig  = pin_to_signal.get(pin, pin)   # 若无 Wire 映射，用 PIN 号代替
        ports_data.append({
            'pin': pin, 'signal': sig,
            'iostandard': vol, 'drive': drv, 'slew': slew, 'pull': pull,
        })

    # 提取额外命令
    extra_code = expand_roplace_get_ports_lists(
        decode_robei_code(root.get('Code', ''))
    )

    # ── 生成 XDC ──────────────────────────────────────────────────────────
    lines = ['# 由 robei_converter.py 从 .const 自动生成', '']

    # PACKAGE_PIN（一行一个）
    lines.append('# ── 引脚分配 ──────────────────────────────────────')
    for p in ports_data:
        if p['pin']:
            lines.append(
                f'set_property PACKAGE_PIN {p["pin"]} [get_ports {p["signal"]}]'
            )

    # Robei's Roplace parser accepts one get_ports object per command. Keep
    # these properties ungrouped even though Vivado also accepts brace lists.
    lines.append('')
    lines.append('# ── IO 属性 ───────────────────────────────────────')
    for p in ports_data:
        signal = p['signal']
        if p['iostandard']:
            lines.append(
                f'set_property IOSTANDARD {p["iostandard"]} [get_ports {signal}]'
            )
        if p['drive']:
            lines.append(f'set_property DRIVE {p["drive"]} [get_ports {signal}]')
        if p['slew']:
            lines.append(f'set_property SLEW {p["slew"]} [get_ports {signal}]')
        if p['pull'] == 'up':
            lines.append(f'set_property PULLUP true [get_ports {signal}]')
        elif p['pull'] == 'down':
            lines.append(f'set_property PULLDOWN true [get_ports {signal}]')

    # 额外命令
    if extra_code.strip():
        lines.append('')
        lines.append('# ── 时序约束等额外命令 ────────────────────────────')
        lines.append(extra_code.strip())

    return '\n'.join(lines) + '\n'


# ═════════════════════════════════════════════════════════════════════════════
# 第五部分：命令行入口
# ═════════════════════════════════════════════════════════════════════════════

def cmd_v2model(args):
    with open(args.input, 'r', encoding='utf-8') as f:
        v_text = f.read()
    result = verilog_to_model(v_text)
    out = args.output or os.path.splitext(args.input)[0] + '.model'
    with open(out, 'w', encoding='utf-8') as f:
        f.write(result)
    print(f'[OK] Verilog → model : {os.path.abspath(out)}')


def cmd_model2v(args):
    with open(args.input, 'r', encoding='utf-8') as f:
        xml_text = f.read()
    result = model_to_verilog(xml_text)
    out = args.output or os.path.splitext(args.input)[0] + '.v'
    with open(out, 'w', encoding='utf-8') as f:
        f.write(result)
    print(f'[OK] model → Verilog : {os.path.abspath(out)}')


def cmd_xdc2const(args):
    with open(args.input, 'r', encoding='utf-8') as f:
        xdc_text = f.read()
    top = args.top or os.path.splitext(os.path.basename(args.input))[0]
    result = xdc_to_const(xdc_text,
                          top_module_name=top,
                          device=args.device,
                          family=args.family,
                          vendor=args.vendor)
    out = args.output or os.path.splitext(args.input)[0] + '.const'
    with open(out, 'w', encoding='utf-8') as f:
        f.write(result)
    print(f'[OK] XDC → const : {os.path.abspath(out)}')


def cmd_const2xdc(args):
    with open(args.input, 'r', encoding='utf-8') as f:
        const_xml = f.read()
    result = const_to_xdc(const_xml)
    out = args.output or os.path.splitext(args.input)[0] + '.xdc'
    with open(out, 'w', encoding='utf-8') as f:
        f.write(result)
    print(f'[OK] const → XDC : {os.path.abspath(out)}')


def main():
    parser = argparse.ArgumentParser(
        description='Robei 可视化工具格式双向转换器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
转换模式：
  v2model    Verilog (.v) → Robei 模块 (.model)
  model2v    Robei 模块 (.model) → Verilog (.v)
  xdc2const  XDC 约束 (.xdc) → Robei 约束 (.const)
  const2xdc  Robei 约束 (.const) → XDC 约束 (.xdc)

示例：
  python robei_converter.py v2model   buzzer_ctrl.v
  python robei_converter.py model2v   buzzer_ctrl.model
  python robei_converter.py xdc2const top.xdc --top top
  python robei_converter.py const2xdc top.const -o top_out.xdc
        """
    )
    subs = parser.add_subparsers(dest='mode', required=True)

    # v2model
    p = subs.add_parser('v2model', help='Verilog (.v) → Robei 模块 (.model)')
    p.add_argument('input',  help='输入 .v 文件路径')
    p.add_argument('-o', '--output', help='输出 .model 路径（默认与输入同名）')
    p.set_defaults(func=cmd_v2model)

    # model2v
    p = subs.add_parser('model2v', help='Robei 模块 (.model) → Verilog (.v)')
    p.add_argument('input',  help='输入 .model 文件路径')
    p.add_argument('-o', '--output', help='输出 .v 路径（默认与输入同名）')
    p.set_defaults(func=cmd_model2v)

    # xdc2const
    p = subs.add_parser('xdc2const', help='XDC 约束 (.xdc) → Robei 约束 (.const)')
    p.add_argument('input',   help='输入 .xdc 文件路径')
    p.add_argument('-o', '--output',  help='输出 .const 路径（默认与输入同名）')
    p.add_argument('--top',    default=None,            help='顶层模块名（默认取文件名）')
    p.add_argument('--device', default='xc7a50tfgg484', help='器件型号（默认 xc7a50tfgg484）')
    p.add_argument('--family', default='artix7',        help='器件族（默认 artix7）')
    p.add_argument('--vendor', default='AMD',           help='厂商（默认 AMD）')
    p.set_defaults(func=cmd_xdc2const)

    # const2xdc
    p = subs.add_parser('const2xdc', help='Robei 约束 (.const) → XDC 约束 (.xdc)')
    p.add_argument('input',  help='输入 .const 文件路径')
    p.add_argument('-o', '--output', help='输出 .xdc 路径（默认与输入同名）')
    p.set_defaults(func=cmd_const2xdc)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
