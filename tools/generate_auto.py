#!/usr/bin/env python3
"""
PathPlanner auto generator.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

AUTO_VERSION = "2026.0"

NAMED_PREPARE = "Prepare Intake"
NAMED_STOP_INTAKE = "Stop Intake"
NAMED_SHOOT = "Shoot Hub"
NAMED_LOWER = "Lower Hood And Stop Shooting"
NAMED_SET_COAST = "Set Coast"
NAMED_TUNABLE_WAIT = "Tunable Wait"
NAMED_IDLE_SHOOTER = "Idle Shooter"

A_SWEEP_PATHS = {
    "MidA": "Pass2_MidSweepA",
    "CloseA": "Pass2_CloseSweepA",
    "FarA": "Pass2_FarSweepA",
    "RiskA": "Pass2_RiskSweepA",
}

B_SWEEP_PATHS = {
    "MidB": "Pass2_MidSweepB",
    "CloseB": "Pass2_CloseSweepB",
    "FarB": "Pass2_FarSweepB",
    "RiskB": "Pass2_RiskSweepB",
}

DOT_PATHS = {
    "center": "Shoot - Center",
    "close": "Shoot - Close",
}

ZERO_DOT_PATHS = {
    "center": "Close - Center",
    "close": "Close - Close",
}

SNEAKY_DOT_PATHS = {
    "center": "Sneaky - Center",
    "close": "Sneaky - Close",
}

AUTO_FOLDERS = {
    "sneaky-dot": "0 - Sneaky Autos",
    "sneaky-sweep": "0 - Sneaky Autos",
    "zero-dot": "1 - Zero Pass Autos",
    "one-pass-dot": "2 - One Pass Dot Autos",
    "one-pass-final": "3 - One Pass Final Sweep Autos",
    "two-pass-dot": "4 - Two Pass Dot Autos",
    "two-pass-final": "5 - Two Pass Final Sweep Autos",
    "manual": "90 - Manual Autos",
}

def named(name: str) -> Dict[str, Any]:
    return {"type": "named", "data": {"name": name}}


def path_cmd(path_name: str) -> Dict[str, Any]:
    return {"type": "path", "data": {"pathName": path_name}}


def sequential(commands: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    return {"type": "sequential", "data": {"commands": list(commands)}}


def make_auto(commands: Sequence[Dict[str, Any]], folder: str, reset_odom: bool = True) -> Dict[str, Any]:
    return {
        "version": AUTO_VERSION,
        "command": sequential(commands),
        "resetOdom": reset_odom,
        "folder": folder,
        "choreoAuto": False,
    }


def is_path_cmd(cmd: Dict[str, Any]) -> bool:
    return isinstance(cmd, dict) and cmd.get("type") == "path"


def get_path_name(cmd: Dict[str, Any]) -> Optional[str]:
    if is_path_cmd(cmd):
        return cmd.get("data", {}).get("pathName")
    return None


def iter_commands(cmd: Dict[str, Any]) -> Iterable[Dict[str, Any]]:
    yield cmd
    if cmd.get("type") in {"sequential", "parallel", "deadline", "race"}:
        for sub in cmd.get("data", {}).get("commands", []):
            yield from iter_commands(sub)


def collect_path_names(cmd: Dict[str, Any]) -> List[str]:
    names: List[str] = []
    for c in iter_commands(cmd):
        path_name = get_path_name(c)
        if path_name:
            names.append(path_name)
    return names


def rewrite_path_names(cmd: Dict[str, Any], mapping: Dict[str, str]) -> None:
    for c in iter_commands(cmd):
        path_name = get_path_name(c)
        if path_name and path_name in mapping:
            c["data"]["pathName"] = mapping[path_name]

def is_pathplanner_project(path: Path) -> bool:
    return (
        (path / "autos").is_dir()
        and (path / "paths").is_dir()
        and (path / "settings.json").is_file()
    )


def resolve_project(path: Path) -> Path:
    raw_path = path.expanduser()

    candidates = [
        raw_path,
        Path.cwd() / raw_path,
        Path(__file__).resolve().parent / raw_path,
    ]

    for candidate in candidates:
        resolved = candidate.resolve()
        if is_pathplanner_project(resolved):
            return resolved

        nested = resolved / "src" / "main" / "deploy" / "pathplanner"
        if is_pathplanner_project(nested):
            return nested.resolve()

    raise FileNotFoundError(f"Could not find a PathPlanner project at {path}")


def paths_dir(project: Path) -> Path:
    return project / "paths"


def autos_dir(project: Path) -> Path:
    return project / "autos"


def path_file(project: Path, path_name: str) -> Path:
    return paths_dir(project) / f"{path_name}.path"


def auto_file(project: Path, auto_name: str) -> Path:
    return autos_dir(project) / f"{auto_name}.auto"


def load_json(file: Path) -> Dict[str, Any]:
    with file.open("r", encoding="utf-8") as f:
        return json.load(f)


def write_json(file: Path, data: Dict[str, Any]) -> None:
    file.parent.mkdir(parents=True, exist_ok=True)
    with file.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def list_path_names(project: Path) -> List[str]:
    return sorted(p.stem for p in paths_dir(project).glob("*.path"))


def list_auto_names(project: Path) -> List[str]:
    return sorted(p.stem for p in autos_dir(project).glob("*.auto"))


def require_paths(project: Path, path_names: Iterable[str]) -> None:
    missing = [p for p in path_names if not path_file(project, p).is_file()]
    if missing:
        raise FileNotFoundError("Missing path file(s): " + ", ".join(missing))


def add_folder_to_settings(project: Path, key: str, folder: str) -> None:
    settings_path = project / "settings.json"
    settings = load_json(settings_path)
    folders = settings.setdefault(key, [])
    if folder and folder not in folders:
        folders.append(folder)
        write_json(settings_path, settings)


def safe_filename(name: str) -> str:
    cleaned = re.sub(r'[\\/:*?"<>|]+', "_", name).strip()
    cleaned = re.sub(r"\s+", " ", cleaned)
    return cleaned or "Generated Auto"


# recipe builder

def first_pass_paths(risk: bool, greedy: bool, hub: bool) -> Tuple[str, str, str, str]:
    side = "Risk" if risk else "Safe"
    greed = "Greedy" if greedy else ""
    hub_text = "Hub" if hub else "NoHub"
    readable = f"P1 {'Risk' if risk else 'Safe'} {'Greedy' if greedy else 'NG'} {'Hub' if hub else 'NoHub'}"
    start = f"Pass1_{side}_Start"
    sweep = f"Pass1_{greed}{side}_Sweep"
    return_path = f"Pass1_{greed}{side}{hub_text}_Return"
    return readable, start, sweep, return_path


def parse_chain_label(label: str, greedy: bool = False) -> Tuple[str, List[str], str]:
    """Return (readable label, path sequence, end token such as MidA/RiskB)."""
    label = label.strip()
    if not label:
        raise ValueError("empty sweep chain")

    explicit_greedy = label.startswith("Greedy_")
    clean_label = label[len("Greedy_"):] if explicit_greedy else label
    use_greedy = greedy or explicit_greedy

    if clean_label in A_SWEEP_PATHS:
        readable = ("Greedy " if use_greedy else "") + clean_label
        # A-only has no connector, so greediness does not change the path name.
        return readable, [A_SWEEP_PATHS[clean_label]], clean_label

    match = re.fullmatch(r"(MidA|CloseA|FarA|RiskA)-(MidB|CloseB|FarB|RiskB)", clean_label)
    if not match:
        raise ValueError(
            f"Could not parse sweep chain '{label}'. "
            "Use MidA, RiskA, MidA-RiskB, Greedy_MidA-RiskB, etc."
        )

    a_token, b_token = match.groups()
    connector = f"{a_token}-{b_token}"
    if use_greedy:
        connector = f"Greedy_{connector}"
    readable = ("Greedy " if use_greedy else "") + f"{a_token}-{b_token}"
    return readable, [A_SWEEP_PATHS[a_token], connector, B_SWEEP_PATHS[b_token]], b_token


def return_paths_for_second(end_token: str, localize: bool) -> List[str]:
    loc = "Localize" if localize else "NoLocalize"
    if end_token.endswith("B"):
        return [f"{end_token}-{loc}", f"Pass2_{loc}_Return"]
    return [f"Pass2_{loc}_Return"]


def build_recipe_commands(args: argparse.Namespace) -> Tuple[List[Dict[str, Any]], str]:
    kind = args.kind
    commands: List[Dict[str, Any]] = []

    if kind == "manual":
        if not args.manual:
            raise ValueError("manual kind requires --manual")
        return parse_manual_sequence(args.manual), args.folder or AUTO_FOLDERS[kind]

    if kind == "sneaky-dot":
        commands += [named(NAMED_TUNABLE_WAIT), path_cmd(SNEAKY_DOT_PATHS[args.dot]), named(NAMED_IDLE_SHOOTER)]
        return commands, args.folder or AUTO_FOLDERS[kind]

    if kind == "zero-dot":
        commands += [path_cmd(ZERO_DOT_PATHS[args.dot]), named(NAMED_IDLE_SHOOTER)]
        return commands, args.folder or AUTO_FOLDERS[kind]

    if kind == "sneaky-sweep":
        _sweep_label, sweep_paths, _end = parse_chain_label(args.second or args.final or "MidA", greedy=args.greedy)
        commands += [named(NAMED_TUNABLE_WAIT), path_cmd("Pass1 Sneaky Start"), named(NAMED_PREPARE)]
        commands += [path_cmd(p) for p in sweep_paths]
        commands += [named(NAMED_STOP_INTAKE), named(NAMED_IDLE_SHOOTER)]
        return commands, args.folder or AUTO_FOLDERS[kind]

    _first_label, start, sweep, return_path = first_pass_paths(
        risk=args.first == "risk",
        greedy=args.first_greedy,
        hub=args.first_hub,
    )
    commands += [
        named(NAMED_PREPARE),
        path_cmd(start),
        path_cmd(sweep),
        path_cmd(return_path),
        named(NAMED_SHOOT),
        named(NAMED_LOWER),
    ]

    if kind == "one-pass-dot":
        commands += [path_cmd(DOT_PATHS[args.dot])]
        return commands, args.folder or AUTO_FOLDERS[kind]

    if kind == "one-pass-final":
        _final_label, final_paths, _end = parse_chain_label(args.final or "MidA", greedy=args.final_greedy)
        commands += [named(NAMED_PREPARE), path_cmd("Pass_Start"), named(NAMED_SET_COAST)]
        commands += [path_cmd(p) for p in final_paths]
        commands += [named(NAMED_STOP_INTAKE)]
        return commands, args.folder or AUTO_FOLDERS[kind]

    if kind in {"two-pass-dot", "two-pass-final"}:
        _second_label, second_paths, second_end = parse_chain_label(
            args.second or "MidA-RiskB",
            greedy=args.second_greedy,
        )
        localize = args.localize == "localize"
        commands += [named(NAMED_PREPARE), path_cmd("Pass_Start")]
        commands += [path_cmd(p) for p in second_paths]
        commands += [path_cmd(p) for p in return_paths_for_second(second_end, localize)]
        commands += [named(NAMED_SHOOT), named(NAMED_LOWER)]

        if kind == "two-pass-dot":
            commands += [path_cmd(DOT_PATHS[args.dot])]
            return commands, args.folder or AUTO_FOLDERS[kind]

        _final_label, final_paths, _final_end = parse_chain_label(
            args.final or "MidA",
            greedy=args.final_greedy,
        )
        commands += [named(NAMED_PREPARE), path_cmd("Pass_Start"), named(NAMED_SET_COAST)]
        commands += [path_cmd(p) for p in final_paths]
        commands += [named(NAMED_STOP_INTAKE)]
        return commands, args.folder or AUTO_FOLDERS[kind]

    raise ValueError(f"Unsupported kind: {kind}")


def parse_manual_sequence(text: str) -> List[Dict[str, Any]]:
    commands: List[Dict[str, Any]] = []
    for raw in [p.strip() for p in text.split(",") if p.strip()]:
        if raw.startswith("path:"):
            commands.append(path_cmd(raw[len("path:"):].strip()))
        elif raw.startswith("named:"):
            commands.append(named(raw[len("named:"):].strip()))
        else:
            commands.append(path_cmd(raw))
    return commands

def copy_paths_for_auto(
    project: Path,
    auto_json: Dict[str, Any],
    auto_name: str,
    copy_folder: Optional[str],
    waypoint_prefix: Optional[str],
    overwrite: bool,
) -> Dict[str, str]:
    original_paths: List[str] = []
    for path_name in collect_path_names(auto_json["command"]):
        if path_name not in original_paths:
            original_paths.append(path_name)

    require_paths(project, original_paths)

    safe_auto_name = safe_filename(auto_name)
    path_folder = copy_folder or f"Copy - {safe_auto_name}"
    prefix = waypoint_prefix if waypoint_prefix is not None else f"{auto_name}_"
    path_mapping: Dict[str, str] = {}

    for old_name in original_paths:
        new_name = safe_filename(f"{auto_name} - {old_name}")
        src = path_file(project, old_name)
        dst = path_file(project, new_name)
        if dst.exists() and not overwrite:
            raise FileExistsError(f"Refusing to overwrite existing copied path: {dst.name}. Use --overwrite.")

        data = load_json(src)
        data["folder"] = path_folder

        for waypoint in data.get("waypoints", []):
            linked = waypoint.get("linkedName")
            if linked:
                waypoint["linkedName"] = f"{prefix}{linked}"

        write_json(dst, data)
        path_mapping[old_name] = new_name

    rewrite_path_names(auto_json["command"], path_mapping)
    add_folder_to_settings(project, "pathFolders", path_folder)
    return path_mapping

def write_auto(
    project: Path,
    auto_name: str,
    auto_json: Dict[str, Any],
    overwrite: bool,
) -> Path:
    auto_name = safe_filename(auto_name)
    dst = auto_file(project, auto_name)
    if dst.exists() and not overwrite:
        raise FileExistsError(f"Refusing to overwrite existing auto: {dst.name}. Use --overwrite.")

    write_json(dst, auto_json)
    add_folder_to_settings(project, "autoFolders", auto_json.get("folder", ""))
    return dst


def print_auto_summary(auto_name: str, auto_json: Dict[str, Any], path_mapping: Optional[Dict[str, str]] = None) -> None:
    paths = collect_path_names(auto_json["command"])
    named_commands = [
        c.get("data", {}).get("name")
        for c in iter_commands(auto_json["command"])
        if c.get("type") == "named"
    ]

    print(f"Auto: {auto_name}")
    print(f"Folder: {auto_json.get('folder', '')}")

    print(f"Paths: {len(paths)}")
    for path_name in paths:
        print(f"  - {path_name}")

    print(f"Named commands: {len(named_commands)}")
    for command_name in named_commands:
        print(f"  - {command_name}")

    if path_mapping:
        print("Copied path mapping:")
        for old, new in path_mapping.items():
            print(f"  - {old} -> {new}")

def cmd_list(args: argparse.Namespace) -> int:
    project = resolve_project(Path(args.project))
    print(f"Project: {project}")

    print("\nAutos:")
    for name in list_auto_names(project):
        print(f"  {name}")

    print("\nPaths:")
    for name in list_path_names(project):
        print(f"  {name}")

    return 0

def cmd_generate(args: argparse.Namespace) -> int:
    project = resolve_project(Path(args.project))
    auto_name = safe_filename(args.name)
    commands, folder = build_recipe_commands(args)
    require_paths(project, collect_path_names(sequential(commands)))

    auto_json = make_auto(commands, folder=folder, reset_odom=not args.no_reset_odom)
    path_mapping: Optional[Dict[str, str]] = None

    if args.copy_paths:
        path_mapping = copy_paths_for_auto(
            project=project,
            auto_json=auto_json,
            auto_name=auto_name,
            copy_folder=args.copy_folder,
            waypoint_prefix=args.waypoint_prefix,
            overwrite=args.overwrite,
        )

    if args.dry_run:
        print_auto_summary(auto_name, auto_json, path_mapping)
        print("Dry run only, no auto written.")
        return 0

    written = write_auto(project, auto_name, auto_json, args.overwrite)
    print_auto_summary(auto_name, auto_json, path_mapping)
    print(f"Wrote: {written}")
    return 0



#CLI

def add_generate_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--name", required=True, help="Auto name to write")
    parser.add_argument(
        "--kind",
        required=True,
        choices=[
            "manual",
            "sneaky-dot",
            "sneaky-sweep",
            "zero-dot",
            "one-pass-dot",
            "one-pass-final",
            "two-pass-dot",
            "two-pass-final",
        ],
    )
    parser.add_argument("--folder", help="PathPlanner GUI auto folder override")
    parser.add_argument("--manual", help="Comma list, e.g. 'named:Prepare Intake,path:Pass1_Safe_Start,path:Pass1_Safe_Sweep'")
    parser.add_argument("--first", choices=["safe", "risk"], default="safe", help="First-pass start/sweep family")
    parser.add_argument("--first-greedy", action="store_true", help="Use greedy first-pass sweep")
    parser.add_argument("--first-hub", action=argparse.BooleanOptionalAction, default=True, help="Use hub sweep return for first pass")
    parser.add_argument("--second", help="Second-pass chain, e.g. MidA, MidA-RiskB, Greedy_MidA-RiskB")
    parser.add_argument("--second-greedy", action="store_true", help="Use greedy connector for --second")
    parser.add_argument("--localize", choices=["localize", "nolocalize"], default="localize")
    parser.add_argument("--final", help="Final sweep chain, e.g. MidA, RiskA-CloseB")
    parser.add_argument("--final-greedy", action="store_true", help="Use greedy connector for --final")
    parser.add_argument("--greedy", action="store_true", help="Convenience greedy flag for sneaky-sweep")
    parser.add_argument("--dot", choices=["center", "close"], default="center")
    parser.add_argument("--copy-paths", action="store_true", help="Copy every path used by this auto and rewrite the auto to use the copies")
    parser.add_argument("--copy-folder", help="PathPlanner GUI folder for copied paths. Default: Copy - <auto name>")
    parser.add_argument("--waypoint-prefix", help="Prefix for copied linked waypoints. Default: <auto name>_")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing auto/copies")
    parser.add_argument("--dry-run", action="store_true", help="Print what would be generated without writing the auto")
    parser.add_argument("--no-reset-odom", action="store_true", help="Set resetOdom false")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate PathPlanner autos and path-copy variants.")
    parser.add_argument(
        "--project",
        default="../src/main/deploy/pathplanner",
        help="PathPlanner folder or repo root",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="List available autos and paths")
    list_parser.set_defaults(func=cmd_list)

    gen_parser = subparsers.add_parser("generate", help="Generate one auto")
    add_generate_args(gen_parser)
    gen_parser.set_defaults(func=cmd_generate)


    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        return args.func(args)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
