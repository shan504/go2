#!/usr/bin/env python3
"""Offline navigation fault inspection/export; works after Docker has stopped."""
import argparse
import json
from pathlib import Path
import sys

from fault_store import (_owned_session, export_fault, list_faults,
                         read_json, render_report, safe_display, client_time, load_fault)


def main(argv=None):
    parser = argparse.ArgumentParser(description='查看或导出已保存的导航故障，不发送控制命令')
    parser.add_argument('--root', type=Path,
                        default=Path(__file__).resolve().parents[1]/'runtime/diagnostics')
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('list', help='列出保存的故障')
    commands.add_parser('sessions', help='列出导航运行记录')
    for name, help_text in (('show', '查看最近或指定故障的原因及证据'),
                            ('export', '导出故障诊断包供后续分析')):
        command = commands.add_parser(name, help=help_text)
        command.add_argument('selector', nargs='?', default='latest')
    args = parser.parse_args(argv)
    try:
        if not args.root.exists():
            if args.command in ('list', 'sessions'):
                print('暂无保存的导航记录。使用新版本启动导航后会自动保存。')
                return 0
            raise ValueError('尚无诊断目录；请先使用新版本启动导航')
        if args.command == 'list':
            faults = list_faults(args.root)
            if not faults:
                print('暂无故障记录。运行会话可能正常，或故障尚未触发自动保存。')
                return 0
            for item in faults:
                trigger = item.get('trigger', {})
                print(safe_display(f"{client_time(item['recorded_utc'])}  {trigger.get('code', 'unknown')}\n"
                      f"  ID: {item['fault_id']}\n"
                      f"  {str(trigger.get('message', ''))[:240]}"))
            print('\n查看: bash go2_3d/tools.sh fault-show ID')
        elif args.command == 'sessions':
            sessions = sorted((p for p in args.root.iterdir() if p.is_dir() and _owned_session(p)), reverse=True)
            for session in sessions:
                manifest = read_json(session/'manifest.json')
                print(safe_display(json.dumps(dict(session_id=manifest.get('session_id', session.name),
                    created_beijing=client_time(manifest.get('created_utc')),
                    ended_beijing=client_time(manifest.get('ended_utc')),
                    created_utc=manifest.get('created_utc'), ended_utc=manifest.get('ended_utc'),
                    exit_code=manifest.get('exit_code')), ensure_ascii=False)))
            if not sessions:
                print('暂无运行会话。')
        elif args.command == 'show':
            selected, evidence, manifest = load_fault(args.root, args.selector)
            fault = Path(selected['fault_path'])
            print(render_report(selected, evidence), end='')
            print('存储保留状态: '+json.dumps(manifest.get('fault_store', {}), ensure_ascii=False))
            print(safe_display('保存目录: '+str(fault)))
            print('导出命令: bash go2_3d/tools.sh fault-export '+selected['fault_id'])
        else:
            exported = export_fault(args.root, args.selector)
            print(safe_display('诊断包已保存: '+str(exported)))
            print('可将此 .tar.gz 文件提供给后续代码分析；它不包含完整 PCD 地图或环境变量。')
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(safe_display('故障记录操作失败: '+str(error)), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
