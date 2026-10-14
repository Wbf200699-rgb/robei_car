#!/usr/bin/env python3
"""Replace Robei Code attributes from Verilog without changing visual layout."""

from pathlib import Path
import argparse
import re


ROOT = Path(__file__).resolve().parents[1]


def encode_code(code: str, newline_entity: str) -> str:
    return (
        code.replace("\r\n", "\n")
        .replace("\r", "\n")
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
        .replace("\n", newline_entity)
    )


def module_body(verilog: str, module_name: str) -> str:
    match = re.search(
        rf"\bmodule\s+{re.escape(module_name)}\s*(?:\(.*?\))?\s*;\s*(.*?)\s*endmodule\b",
        verilog,
        flags=re.S,
    )
    if not match:
        raise SystemExit(f"cannot find module {module_name}")
    return re.sub(
        r"^\s*//----Code starts here: integrated by Robei-----\r?\n",
        "",
        match.group(1),
        count=1,
    ).strip("\r\n")


def replace_code(path: Path, instance_name: str, body: str) -> None:
    text = path.read_text(encoding="utf-8")
    pattern = re.compile(
        # Robei stores escaped Verilog operators such as &gt; inside Code.
        # Match within the physical XML element line so the entity's trailing
        # '>' cannot be mistaken for the end of the <Module> start tag.
        rf'(<Module\b(?=[^\r\n]*\bName="{re.escape(instance_name)}")[^\r\n]*\bCode=")'
        rf'(.*?)(" Color=)',
        flags=re.S,
    )
    match = pattern.search(text)
    if not match:
        raise SystemExit(f"cannot find Code for {instance_name} in {path}")
    old = match.group(2)
    entity = "&#10;" if old.count("&#10;") > old.count("&#xa;") else "&#xa;"
    updated = text[: match.start(2)] + encode_code(body, entity) + text[match.end(2) :]
    path.write_text(updated, encoding="utf-8", newline="\n")
    print(f"synced {path.relative_to(ROOT)}:{instance_name}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("module")
    parser.add_argument("verilog", type=Path)
    parser.add_argument("model", type=Path)
    parser.add_argument("--top-model", type=Path)
    parser.add_argument("--top-instance")
    parser.add_argument("--mirror", type=Path)
    args = parser.parse_args()

    args.verilog = args.verilog.resolve()
    args.model = args.model.resolve()
    if args.top_model:
        args.top_model = args.top_model.resolve()
    if args.mirror:
        args.mirror = args.mirror.resolve()
    verilog = args.verilog.read_text(encoding="utf-8")
    body = module_body(verilog, args.module)
    replace_code(args.model, args.module, body)
    if args.top_model or args.top_instance:
        if not args.top_model or not args.top_instance:
            parser.error("--top-model and --top-instance must be used together")
        replace_code(args.top_model, args.top_instance, body)
    if args.mirror:
        args.mirror.write_text(verilog, encoding="utf-8", newline="\n")
        print(f"synced {args.mirror.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
