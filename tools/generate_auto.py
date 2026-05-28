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



# GUI

def route_to_key(value: str) -> str:
    return "close-far" if value == "Close -> Far" else "far-close"


def chain_from_pass(risky: bool, greedy: bool, hub: bool, route: str) -> str:
    near = "Close" if hub else "Mid"
    far = "Risk" if risky else "Far"
    route = route_to_key(route)
    chain = f"{far}A-{near}B" if route == "far-close" else f"{near}A-{far}B"
    return f"Greedy_{chain}" if greedy else chain


def build_dynamic_commands(passes: List[Dict[str, Any]], final_pass: Optional[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], List[str]]:
    commands: List[Dict[str, Any]] = []
    warnings: List[str] = []
    shot_count = 0

    if passes and passes[0]["start"].get() == "sneaky" and final_pass and final_pass["type"].get() == "dot":
        commands += [named(NAMED_TUNABLE_WAIT), path_cmd(SNEAKY_DOT_PATHS[final_pass["dot"].get()]), named(NAMED_IDLE_SHOOTER)]
        if len(passes) > 1:
            warnings.append("Sneaky dot uses the first pass start only. Other passes were ignored.")
        return commands, warnings

    for index, pass_data in enumerate(passes):
        risky = bool(pass_data["risky"].get())
        greedy = bool(pass_data["greedy"].get())
        hub = bool(pass_data["hub"].get())
        route = pass_data["route"].get()

        if index == 0 and pass_data["start"].get() == "sneaky":
            if len(passes) > 1 or final_pass:
                warnings.append("Sneaky start only runs the sneaky sweep. Later passes/finals were ignored.")
            _label, sweep_paths, _end = parse_chain_label(chain_from_pass(risky, greedy, hub, route))
            commands += [named(NAMED_TUNABLE_WAIT), path_cmd("Pass1 Sneaky Start"), named(NAMED_PREPARE)]
            commands += [path_cmd(p) for p in sweep_paths]
            commands += [named(NAMED_STOP_INTAKE), named(NAMED_IDLE_SHOOTER)]
            return commands, warnings

        if index == 0:
            _label, start, sweep, return_path = first_pass_paths(risky, greedy, hub)
            commands += [named(NAMED_PREPARE), path_cmd(start), path_cmd(sweep), path_cmd(return_path)]
        else:
            _label, sweep_paths, end_token = parse_chain_label(chain_from_pass(risky, greedy, hub, route))
            commands += [named(NAMED_PREPARE), path_cmd("Pass_Start")]
            commands += [path_cmd(p) for p in sweep_paths]
            commands += [path_cmd(p) for p in return_paths_for_second(end_token, bool(pass_data["localize"].get()))]

        commands += [named(NAMED_SHOOT), named(NAMED_LOWER)]
        shot_count += 1

    if final_pass:
        if final_pass["type"].get() == "dot":
            dot = final_pass["dot"].get()
            if shot_count > 0:
                commands += [path_cmd(DOT_PATHS[dot])]
            else:
                commands += [path_cmd(ZERO_DOT_PATHS[dot]), named(NAMED_IDLE_SHOOTER)]
        else:
            risky = bool(final_pass["risky"].get())
            greedy = bool(final_pass["greedy"].get())
            hub = bool(final_pass["hub"].get())
            route = final_pass["route"].get()
            _label, final_paths, _end = parse_chain_label(chain_from_pass(risky, greedy, hub, route))
            commands += [named(NAMED_PREPARE), path_cmd("Pass_Start"), named(NAMED_SET_COAST)]
            commands += [path_cmd(p) for p in final_paths]
            commands += [named(NAMED_STOP_INTAKE)]

    if not commands:
        raise ValueError("Add at least one pass or a final pass.")

    return commands, warnings


def dynamic_preview_text(commands: List[Dict[str, Any]], warnings: List[str], auto_name: str, folder: str) -> str:
    auto_json = make_auto(commands, folder=folder, reset_odom=True)
    paths = collect_path_names(auto_json["command"])
    named_commands = [
        c.get("data", {}).get("name")
        for c in iter_commands(auto_json["command"])
        if c.get("type") == "named"
    ]

    lines = [
        f"Auto: {safe_filename(auto_name) if auto_name else '(not named yet)'}",
        f"Folder: {folder}",
        "Reset odom: True",
    ]

    if warnings:
        lines += ["", "Warnings:"]
        lines += [f"  - {w}" for w in warnings]

    lines += ["", "Path sequence:"]
    lines += [f"  {index + 1:02d}. {p}" for index, p in enumerate(paths)] or ["  None"]
    lines += ["", "Named commands:"]
    lines += [f"  {index + 1:02d}. {c}" for index, c in enumerate(named_commands)] or ["  None"]

    return "\n".join(lines)



def canonical_generated_path(path_name: str) -> str:
    known_paths = set(ZERO_DOT_PATHS.values()) | set(SNEAKY_DOT_PATHS.values()) | set(DOT_PATHS.values())
    known_paths |= set(A_SWEEP_PATHS.values()) | set(B_SWEEP_PATHS.values())
    known_paths |= {
        "Pass1_Safe_Start",
        "Pass1_Risk_Start",
        "Pass1 Sneaky Start",
        "Pass_Start",
        "Pass2_Localize_Return",
        "Pass2_NoLocalize_Return",
    }

    for risk in [False, True]:
        for greedy in [False, True]:
            for hub in [False, True]:
                _readable, start, sweep, return_path = first_pass_paths(risk, greedy, hub)
                known_paths |= {start, sweep, return_path}

    for a in A_SWEEP_PATHS:
        for b in B_SWEEP_PATHS:
            known_paths.add(f"{a}-{b}")
            known_paths.add(f"Greedy_{a}-{b}")
    for b in B_SWEEP_PATHS:
        known_paths.add(f"{b}-Localize")
        known_paths.add(f"{b}-NoLocalize")

    if path_name in known_paths:
        return path_name
    for known in sorted(known_paths, key=len, reverse=True):
        if path_name.endswith(f" - {known}"):
            return known
    return path_name


def top_level_commands(auto_json: Dict[str, Any]) -> List[Dict[str, Any]]:
    command = auto_json.get("command", {})
    if command.get("type") == "sequential":
        return list(command.get("data", {}).get("commands", []))
    return [command] if command else []


def command_named(command: Dict[str, Any], name: str) -> bool:
    return command.get("type") == "named" and command.get("data", {}).get("name") == name


def command_path_name(command: Dict[str, Any]) -> Optional[str]:
    path_name = get_path_name(command)
    return canonical_generated_path(path_name) if path_name else None


def parse_chain_from_paths(paths: List[str]) -> Dict[str, Any]:
    if not paths:
        raise ValueError("missing sweep path")

    inv_a = {path: token for token, path in A_SWEEP_PATHS.items()}
    inv_b = {path: token for token, path in B_SWEEP_PATHS.items()}
    a_token = inv_a.get(paths[0])
    if not a_token:
        raise ValueError(f"expected SweepA path, got {paths[0]}")

    if len(paths) == 1:
        return {
            "risky": a_token == "RiskA",
            "greedy": False,
            "hub": a_token == "CloseA",
            "route": "Far -> Close",
            "end_token": a_token,
        }

    if len(paths) != 3:
        raise ValueError(f"expected A or A-connector-B sweep, got {paths}")

    connector = paths[1]
    b_token = inv_b.get(paths[2])
    if not b_token:
        raise ValueError(f"expected SweepB path, got {paths[2]}")

    greedy = connector.startswith("Greedy_")
    clean = connector[len("Greedy_"):] if greedy else connector
    match = re.fullmatch(r"(MidA|CloseA|FarA|RiskA)-(MidB|CloseB|FarB|RiskB)", clean)
    if not match:
        raise ValueError(f"could not parse connector {connector}")

    start_token, end_token = match.groups()
    if start_token != a_token or end_token != b_token:
        raise ValueError(f"connector {connector} does not match {paths[0]} -> {paths[2]}")

    if a_token in {"FarA", "RiskA"}:
        route = "Far -> Close"
        far_token = a_token
        near_token = b_token
    else:
        route = "Close -> Far"
        far_token = b_token
        near_token = a_token

    return {
        "risky": "Risk" in far_token,
        "greedy": greedy,
        "hub": "Close" in near_token,
        "route": route,
        "end_token": b_token,
    }


def parse_first_pass(start_path: str, sweep_path: str, return_path: str) -> Dict[str, Any]:
    if start_path not in {"Pass1_Safe_Start", "Pass1_Risk_Start"}:
        raise ValueError(f"expected Pass1 start path, got {start_path}")
    risky = start_path == "Pass1_Risk_Start"
    greedy = sweep_path.startswith("Pass1_Greedy")
    hub = "NoHub" not in return_path
    expected = first_pass_paths(risky, greedy, hub)
    if sweep_path != expected[2] or return_path != expected[3]:
        raise ValueError(f"first pass paths do not match: {start_path}, {sweep_path}, {return_path}")
    return {
        "start": "rush",
        "risky": risky,
        "greedy": greedy,
        "hub": hub,
        "route": "Far -> Close",
        "localize": True,
    }


def parse_generated_auto(auto_json: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], Optional[Dict[str, Any]]]:
    commands = top_level_commands(auto_json)
    passes: List[Dict[str, Any]] = []
    final_pass: Optional[Dict[str, Any]] = None
    index = 0

    if len(commands) >= 3 and command_named(commands[0], NAMED_TUNABLE_WAIT):
        path_name = command_path_name(commands[1])
        if path_name in SNEAKY_DOT_PATHS.values():
            dot = next(key for key, value in SNEAKY_DOT_PATHS.items() if value == path_name)
            return ([{"start": "sneaky", "risky": False, "greedy": False, "hub": True, "route": "Far -> Close", "localize": True}], {"type": "dot", "dot": dot})
        if path_name == "Pass1 Sneaky Start" and len(commands) >= 5 and command_named(commands[2], NAMED_PREPARE):
            sweep_paths: List[str] = []
            index = 3
            while index < len(commands) and command_path_name(commands[index]):
                sweep_paths.append(command_path_name(commands[index]) or "")
                index += 1
            spec = parse_chain_from_paths(sweep_paths)
            spec["start"] = "sneaky"
            return ([spec], None)

    while index < len(commands):
        if command_named(commands[index], NAMED_PREPARE):
            next_path = command_path_name(commands[index + 1]) if index + 1 < len(commands) else None

            if next_path in {"Pass1_Safe_Start", "Pass1_Risk_Start"}:
                if index + 5 >= len(commands):
                    raise ValueError("first pass is incomplete")
                start_path = next_path
                sweep_path = command_path_name(commands[index + 2])
                return_path = command_path_name(commands[index + 3])
                if not sweep_path or not return_path:
                    raise ValueError("first pass is missing sweep or return")
                passes.append(parse_first_pass(start_path, sweep_path, return_path))
                if not command_named(commands[index + 4], NAMED_SHOOT) or not command_named(commands[index + 5], NAMED_LOWER):
                    raise ValueError("first pass is missing shoot/lower commands")
                index += 6
                continue

            if next_path == "Pass_Start":
                if index + 2 < len(commands) and command_named(commands[index + 2], NAMED_SET_COAST):
                    sweep_paths = []
                    index += 3
                    while index < len(commands) and command_path_name(commands[index]):
                        sweep_paths.append(command_path_name(commands[index]) or "")
                        index += 1
                    spec = parse_chain_from_paths(sweep_paths)
                    final_pass = {
                        "type": "sweep",
                        "risky": spec["risky"],
                        "greedy": spec["greedy"],
                        "hub": spec["hub"],
                        "route": spec["route"],
                    }
                    if index < len(commands) and command_named(commands[index], NAMED_STOP_INTAKE):
                        index += 1
                    continue

                sweep_paths = []
                index += 2
                while index < len(commands):
                    path_name = command_path_name(commands[index])
                    if not path_name:
                        break
                    if path_name.endswith("-Localize") or path_name.endswith("-NoLocalize") or path_name in {"Pass2_Localize_Return", "Pass2_NoLocalize_Return"}:
                        break
                    sweep_paths.append(path_name)
                    index += 1

                spec = parse_chain_from_paths(sweep_paths)
                localize = True
                if index < len(commands):
                    localize_path = command_path_name(commands[index])
                    if localize_path and localize_path.endswith("-NoLocalize"):
                        localize = False
                    if localize_path and (localize_path.endswith("-Localize") or localize_path.endswith("-NoLocalize")):
                        index += 1
                if index < len(commands):
                    return_path = command_path_name(commands[index])
                    if return_path == "Pass2_NoLocalize_Return":
                        localize = False
                    if return_path in {"Pass2_Localize_Return", "Pass2_NoLocalize_Return"}:
                        index += 1
                if index + 1 >= len(commands) or not command_named(commands[index], NAMED_SHOOT) or not command_named(commands[index + 1], NAMED_LOWER):
                    raise ValueError("second pass is missing shoot/lower commands")
                passes.append({
                    "start": "rush",
                    "risky": spec["risky"],
                    "greedy": spec["greedy"],
                    "hub": spec["hub"],
                    "route": spec["route"],
                    "localize": localize,
                })
                index += 2
                continue

        path_name = command_path_name(commands[index])
        if path_name in DOT_PATHS.values():
            dot = next(key for key, value in DOT_PATHS.items() if value == path_name)
            final_pass = {"type": "dot", "dot": dot}
            index += 1
            continue
        if path_name in ZERO_DOT_PATHS.values():
            dot = next(key for key, value in ZERO_DOT_PATHS.items() if value == path_name)
            final_pass = {"type": "dot", "dot": dot}
            index += 1
            continue

        if command_named(commands[index], NAMED_IDLE_SHOOTER):
            index += 1
            continue

        raise ValueError("custom auto selected, cannot parse into pass editor")

    if not passes:
        raise ValueError("custom auto selected, cannot parse into pass editor")
    return passes, final_pass

def cmd_gui(args: argparse.Namespace) -> int:
    return launch_gui(args.project)


def launch_gui(project_default: str = "../src/main/deploy/pathplanner") -> int:
    try:
        import tkinter as tk
        from tkinter import filedialog, messagebox, scrolledtext, ttk
    except Exception as exc:
        print(f"error: could not load tkinter: {exc}", file=sys.stderr)
        return 1

    root = tk.Tk()
    root.title("PathPlanner Auto Generator")
    root.geometry("1180x820")
    root.minsize(1020, 700)

    bg = "#0f172a"
    panel = "#111827"
    card = "#1f2937"
    raised = "#243244"
    text = "#f8fafc"
    muted = "#94a3b8"
    border = "#334155"
    accent = "#0284c7"
    accent_hover = "#0369a1"
    danger = "#dc2626"
    success = "#16a34a"

    root.configure(bg=bg)

    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except tk.TclError:
        pass
    style.configure("TCombobox", fieldbackground=raised, background=raised, foreground=text, arrowcolor=text, bordercolor=border)
    style.map("TCombobox", fieldbackground=[("readonly", raised)], foreground=[("readonly", text)])

    project_var = tk.StringVar(value=project_default)
    auto_name_var = tk.StringVar(value="New Auto")
    save_copy_var = tk.BooleanVar(value=False)
    overwrite_var = tk.BooleanVar(value=False)
    final_pass: Optional[Dict[str, Any]] = None
    pass_rows: List[Dict[str, Any]] = []
    loaded_custom_auto = tk.StringVar(value="")

    def label(parent: Any, value: str, size: int = 10, weight: str = "normal", fg: str = text, bg_color: str = card) -> tk.Label:
        return tk.Label(parent, text=value, bg=bg_color, fg=fg, font=("Segoe UI", size, weight))

    def button(parent: Any, value: str, command: Any, bg_color: str = raised, fg: str = text, padx: int = 14) -> tk.Button:
        return tk.Button(
            parent,
            text=value,
            command=command,
            bg=bg_color,
            fg=fg,
            activebackground=accent_hover if bg_color == accent else bg_color,
            activeforeground=fg,
            relief="flat",
            bd=0,
            highlightthickness=0,
            padx=padx,
            pady=8,
            cursor="hand2",
            font=("Segoe UI", 10, "bold"),
        )

    def card_frame(parent: Any, bg_color: str = card) -> tk.Frame:
        return tk.Frame(parent, bg=bg_color, highlightbackground=border, highlightthickness=1, bd=0)

    def check(parent: Any, value: str, variable: tk.BooleanVar, bg_color: str = card) -> tk.Checkbutton:
        return tk.Checkbutton(
            parent,
            text=value,
            variable=variable,
            bg=bg_color,
            fg=text,
            activebackground=bg_color,
            activeforeground=text,
            selectcolor=bg,
            relief="flat",
            bd=0,
            highlightthickness=0,
            font=("Segoe UI", 10),
        )

    def radio(parent: Any, value: str, variable: tk.StringVar, option: str, bg_color: str = card) -> tk.Radiobutton:
        return tk.Radiobutton(
            parent,
            text=value,
            variable=variable,
            value=option,
            bg=bg_color,
            fg=text,
            activebackground=bg_color,
            activeforeground=text,
            selectcolor=bg,
            relief="flat",
            bd=0,
            highlightthickness=0,
            font=("Segoe UI", 10),
        )

    def combo(parent: Any, variable: tk.StringVar, values: Sequence[str], width: int = 16) -> ttk.Combobox:
        return ttk.Combobox(parent, textvariable=variable, values=list(values), width=width, state="readonly")

    app = tk.Frame(root, bg=bg)
    app.pack(fill="both", expand=True, padx=16, pady=12)
    app.columnconfigure(0, weight=1)
    app.rowconfigure(1, weight=1)

    header = tk.Frame(app, bg=bg)
    header.grid(row=0, column=0, sticky="ew", pady=(0, 10))
    header.columnconfigure(0, weight=1)

    title_box = tk.Frame(header, bg=bg)
    title_box.grid(row=0, column=0, sticky="w")
    label(title_box, "PathPlanner Auto Generator", 23, "bold", text, bg).pack(anchor="w")
    label(title_box, "Add passes, choose a final path, preview when needed, then save.", 10, "normal", muted, bg).pack(anchor="w", pady=(3, 0))

    header_actions = tk.Frame(header, bg=bg)
    header_actions.grid(row=0, column=1, sticky="e")

    toolbar = card_frame(app, panel)
    toolbar.grid(row=1, column=0, sticky="ew", pady=(0, 8))
    toolbar_inner = tk.Frame(toolbar, bg=panel)
    toolbar_inner.pack(fill="x", padx=12, pady=12)

    canvas = tk.Canvas(app, bg=bg, highlightthickness=0, bd=0)
    scrollbar = tk.Scrollbar(app, orient="vertical", command=canvas.yview, bg=bg, troughcolor=bg, relief="flat", bd=0)
    pass_frame = tk.Frame(canvas, bg=bg)
    pass_frame_id = canvas.create_window((0, 0), window=pass_frame, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar.set)
    canvas.grid(row=2, column=0, sticky="nsew")
    scrollbar.grid(row=2, column=1, sticky="ns")
    app.rowconfigure(2, weight=1)

    status = card_frame(app, panel)
    status.grid(row=3, column=0, sticky="ew", pady=(8, 0))
    status_inner = tk.Frame(status, bg=panel)
    status_inner.pack(fill="x", padx=12, pady=8)
    status_label = label(status_inner, "Ready", 10, "normal", muted, panel)
    status_label.pack(anchor="w")

    def configure_scroll(_event: Any = None) -> None:
        canvas.configure(scrollregion=canvas.bbox("all"))
        canvas.itemconfigure(pass_frame_id, width=canvas.winfo_width())

    def on_mousewheel(event: Any) -> None:
        canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    pass_frame.bind("<Configure>", configure_scroll)
    canvas.bind("<Configure>", configure_scroll)
    canvas.bind_all("<MouseWheel>", on_mousewheel)

    def get_folder_for_save(name: str) -> str:
        return safe_filename(name)

    def current_commands(name: str = "") -> Tuple[List[Dict[str, Any]], List[str], str]:
        commands, warnings = build_dynamic_commands(pass_rows, final_pass)
        folder = get_folder_for_save(name or auto_name_var.get())
        return commands, warnings, folder

    def current_preview() -> str:
        commands, warnings, folder = current_commands()
        return dynamic_preview_text(commands, warnings, auto_name_var.get(), folder)

    def update_status(*_args: Any) -> None:
        try:
            commands, warnings, _folder = current_commands()
            paths = collect_path_names(sequential(commands))
            named_commands = [c for c in iter_commands(sequential(commands)) if c.get("type") == "named"]
            message = f"{len(pass_rows)} pass(es)"
            if final_pass:
                message += f" + final {final_pass['type'].get()}"
            message += f"  |  {len(paths)} paths  |  {len(named_commands)} commands"
            if warnings:
                message += f"  |  {len(warnings)} warning(s)"
            if loaded_custom_auto.get():
                message = loaded_custom_auto.get()
            status_label.configure(text=message)
        except Exception as exc:
            status_label.configure(text=f"error: {exc}")

    def show_text_window(title: str, value: str) -> None:
        dialog = tk.Toplevel(root)
        dialog.title(title)
        dialog.configure(bg=bg)
        dialog.geometry("760x560")
        dialog.minsize(640, 420)
        frame = card_frame(dialog, panel)
        frame.pack(fill="both", expand=True, padx=18, pady=18)
        label(frame, title, 16, "bold", text, panel).pack(anchor="w", padx=14, pady=(14, 8))
        box = scrolledtext.ScrolledText(frame, wrap="word", borderwidth=0, relief="flat")
        box.pack(fill="both", expand=True, padx=14, pady=(0, 14))
        box.configure(bg="#020617", fg="#dbeafe", insertbackground=text, font=("Cascadia Mono", 10), padx=14, pady=14)
        box.insert("1.0", value)
        box.configure(state="disabled")

    def show_preview() -> None:
        try:
            show_text_window("Preview", current_preview())
        except Exception as exc:
            messagebox.showerror("Preview failed", str(exc))

    def sample_path(path_json: Dict[str, Any], samples_per_segment: int = 40) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = []
        waypoints = path_json.get("waypoints", [])
        for segment_index in range(len(waypoints) - 1):
            start = waypoints[segment_index]
            end = waypoints[segment_index + 1]
            p0 = start["anchor"]
            p3 = end["anchor"]
            p1 = start.get("nextControl") or p0
            p2 = end.get("prevControl") or p3
            first_sample = 0 if segment_index == 0 else 1
            for sample in range(first_sample, samples_per_segment + 1):
                t = sample / samples_per_segment
                u = 1.0 - t
                x = (
                    u * u * u * p0["x"]
                    + 3 * u * u * t * p1["x"]
                    + 3 * u * t * t * p2["x"]
                    + t * t * t * p3["x"]
                )
                y = (
                    u * u * u * p0["y"]
                    + 3 * u * u * t * p1["y"]
                    + 3 * u * t * t * p2["y"]
                    + t * t * t * p3["y"]
                )
                points.append((x, y))
        return points

    def show_visualization() -> None:
        try:
            project = resolve_project(Path(project_var.get()))
            commands, warnings, _folder = current_commands()
            path_names = collect_path_names(sequential(commands))
            if not path_names:
                messagebox.showinfo("Visualize", "This auto has no paths to draw.")
                return

            loaded_paths: List[Tuple[str, List[Tuple[float, float]]]] = []
            missing: List[str] = []
            for path_name in path_names:
                file = path_file(project, path_name)
                if not file.is_file():
                    missing.append(path_name)
                    continue
                loaded_paths.append((path_name, sample_path(load_json(file), samples_per_segment=70)))

            if missing:
                messagebox.showerror("Visualize failed", "Missing path file(s): " + ", ".join(missing))
                return

            playback_points: List[Tuple[float, float, int]] = []
            for path_index, (_path_name, points) in enumerate(loaded_paths):
                for point in points:
                    playback_points.append((point[0], point[1], path_index))

            if not playback_points:
                messagebox.showinfo("Visualize", "No drawable points were found.")
                return

            script_dir = Path(__file__).resolve().parent
            navgrid_candidates = [script_dir / "navgrid.json", project / "navgrid.json"]
            field_image_candidates = [script_dir / "BattlecryField26.png", project / "BattlecryField26.png"]

            field_size = {"x": 16.54, "y": 8.07}
            navgrid_file = next((file for file in navgrid_candidates if file.is_file()), None)
            if navgrid_file is not None:
                try:
                    field_size = load_json(navgrid_file).get("field_size", field_size)
                except Exception:
                    pass

            field_x = float(field_size.get("x") or 16.54)
            field_y = float(field_size.get("y") or 8.07)
            field_image_file = next((file for file in field_image_candidates if file.is_file()), None)
            field_image_source = None
            field_image_error = None
            image_size = None

            if field_image_file is not None:
                try:
                    from PIL import Image, ImageTk
                    field_image_source = Image.open(field_image_file).convert("RGBA")
                    image_size = field_image_source.size
                except Exception as exc:
                    field_image_error = str(exc)

            dialog = tk.Toplevel(root)
            dialog.title("Auto Playback")
            dialog.configure(bg=bg)
            dialog.geometry("1200x760")
            dialog.minsize(940, 600)

            outer = card_frame(dialog, panel)
            outer.pack(fill="both", expand=True, padx=18, pady=18)
            top = tk.Frame(outer, bg=panel)
            top.pack(fill="x", padx=14, pady=(14, 8))
            label(top, "Auto Playback", 17, "bold", text, panel).pack(side="left")
            current_path_var = tk.StringVar(value="Ready")
            label(top, "Scrub, play, or pause through the generated path sequence.", 10, "normal", muted, panel).pack(side="left", padx=(14, 0))
            label(top, f"{len(path_names)} path(s)", 10, "normal", muted, panel).pack(side="right")

            body = tk.Frame(outer, bg=panel)
            body.pack(fill="both", expand=True, padx=14, pady=(0, 10))
            body.columnconfigure(0, weight=1)
            body.rowconfigure(0, weight=1)

            field = tk.Canvas(body, bg="#020617", highlightthickness=1, highlightbackground=border, bd=0)
            field.grid(row=0, column=0, sticky="nsew", padx=(0, 12))

            sidebar = scrolledtext.ScrolledText(body, width=34, wrap="word", borderwidth=0, relief="flat")
            sidebar.grid(row=0, column=1, sticky="ns")
            sidebar.configure(bg="#020617", fg="#dbeafe", insertbackground=text, font=("Cascadia Mono", 9), padx=12, pady=12)

            controls = card_frame(outer, panel)
            controls.pack(fill="x", padx=14, pady=(0, 14))
            controls_inner = tk.Frame(controls, bg=panel)
            controls_inner.pack(fill="x", padx=10, pady=10)

            playing = tk.BooleanVar(value=False)
            frame_var = tk.IntVar(value=0)
            speed_var = tk.StringVar(value="1x")
            image_ref: Dict[str, Any] = {}
            colors = [
                "#38bdf8", "#fb7185", "#4ade80", "#facc15", "#c084fc", "#f97316",
                "#22d3ee", "#a3e635", "#f472b6", "#60a5fa", "#fde68a", "#34d399",
            ]

            def get_view(width: int, height: int) -> Tuple[float, float, float, float, float]:
                pad = 18
                if image_size:
                    image_w, image_h = image_size
                    scale = min((width - pad * 2) / image_w, (height - pad * 2) / image_h)
                    drawn_w = image_w * scale
                    drawn_h = image_h * scale
                else:
                    scale = min((width - pad * 2) / field_x, (height - pad * 2) / field_y)
                    drawn_w = field_x * scale
                    drawn_h = field_y * scale
                left = (width - drawn_w) / 2
                top = (height - drawn_h) / 2
                return left, top, drawn_w, drawn_h, scale

            def transform(x: float, y: float, width: int, height: int) -> Tuple[float, float]:
                left, top, drawn_w, drawn_h, _scale = get_view(width, height)
                return left + (x / field_x) * drawn_w, top + ((field_y - y) / field_y) * drawn_h

            def draw_background(width: int, height: int) -> None:
                left, top, drawn_w, drawn_h, _scale = get_view(width, height)
                if field_image_source is not None:
                    resized = field_image_source.resize((max(1, int(drawn_w)), max(1, int(drawn_h))))
                    image_ref["image"] = ImageTk.PhotoImage(resized)
                    field.create_image(left, top, anchor="nw", image=image_ref["image"])
                    field.create_rectangle(left, top, left + drawn_w, top + drawn_h, outline="#94a3b8", width=1)
                    return

                field.create_rectangle(left, top, left + drawn_w, top + drawn_h, fill="#020617", outline="#334155", width=1)
                for gx in range(0, int(field_x) + 1):
                    x1, y1 = transform(gx, 0.0, width, height)
                    x2, y2 = transform(gx, field_y, width, height)
                    field.create_line(x1, y1, x2, y2, fill="#1e293b")
                for gy in range(0, int(field_y) + 1):
                    x1, y1 = transform(0.0, gy, width, height)
                    x2, y2 = transform(field_x, gy, width, height)
                    field.create_line(x1, y1, x2, y2, fill="#1e293b")

            def draw_frame() -> None:
                width = max(field.winfo_width(), 300)
                height = max(field.winfo_height(), 260)
                frame = min(max(frame_var.get(), 0), len(playback_points) - 1)
                frame_var.set(frame)
                active_path = playback_points[frame][2]
                field.delete("all")
                draw_background(width, height)

                for index, (path_name, points) in enumerate(loaded_paths):
                    if len(points) < 2:
                        continue
                    color = colors[index % len(colors)]
                    coords: List[float] = []
                    for x, y in points:
                        sx, sy = transform(x, y, width, height)
                        coords += [sx, sy]
                    field.create_line(*coords, fill=color if index == active_path else "#475569", width=4 if index == active_path else 2, smooth=True)
                    sx, sy = transform(*points[0], width, height)
                    ex, ey = transform(*points[-1], width, height)
                    field.create_oval(sx - 5, sy - 5, sx + 5, sy + 5, fill=color, outline="")
                    field.create_oval(ex - 4, ey - 4, ex + 4, ey + 4, fill="#020617", outline=color, width=2)
                    field.create_text(sx + 9, sy - 9, text=str(index + 1), fill=color, anchor="w", font=("Segoe UI", 10, "bold"))

                trail_points = playback_points[:frame + 1]
                trail_coords: List[float] = []
                for x, y, _path_index in trail_points[-260:]:
                    sx, sy = transform(x, y, width, height)
                    trail_coords += [sx, sy]
                if len(trail_coords) >= 4:
                    field.create_line(*trail_coords, fill="#f8fafc", width=3, smooth=True)

                x, y, path_index = playback_points[frame]
                sx, sy = transform(x, y, width, height)
                if frame > 0:
                    px, py, _prev_path = playback_points[frame - 1]
                    psx, psy = transform(px, py, width, height)
                    field.create_line(psx, psy, sx, sy, fill="#f8fafc", width=5, arrow=tk.LAST, arrowshape=(16, 20, 7))
                field.create_oval(sx - 10, sy - 10, sx + 10, sy + 10, fill="#f8fafc", outline="#0284c7", width=3)
                field.create_oval(sx - 3, sy - 3, sx + 3, sy + 3, fill="#0284c7", outline="")
                current_path_var.set(f"{frame + 1}/{len(playback_points)}  |  {path_names[path_index]}")

            def step_amount() -> int:
                return {"0.25x": 1, "0.5x": 2, "1x": 4, "2x": 8, "4x": 14}.get(speed_var.get(), 4)

            def play_loop() -> None:
                if not playing.get():
                    return
                next_frame = frame_var.get() + step_amount()
                if next_frame >= len(playback_points):
                    next_frame = len(playback_points) - 1
                    playing.set(False)
                    play_button.configure(text="Play")
                frame_var.set(next_frame)
                draw_frame()
                if playing.get():
                    dialog.after(33, play_loop)

            def toggle_play() -> None:
                playing.set(not playing.get())
                play_button.configure(text="Pause" if playing.get() else "Play")
                if playing.get():
                    play_loop()

            def restart() -> None:
                playing.set(False)
                play_button.configure(text="Play")
                frame_var.set(0)
                draw_frame()

            def slider_changed(_value: str) -> None:
                draw_frame()

            play_button = button(controls_inner, "Play", toggle_play, accent)
            play_button.pack(side="left", padx=(0, 8))
            button(controls_inner, "Restart", restart, raised).pack(side="left", padx=(0, 12))
            label(controls_inner, "Speed", 10, "normal", muted, panel).pack(side="left", padx=(0, 6))
            speed_box = combo(controls_inner, speed_var, ["0.25x", "0.5x", "1x", "2x", "4x"], width=8)
            speed_box.pack(side="left", padx=(0, 12))
            label(controls_inner, "Frame", 10, "normal", muted, panel).pack(side="left", padx=(0, 8))
            slider = tk.Scale(
                controls_inner,
                from_=0,
                to=max(0, len(playback_points) - 1),
                orient="horizontal",
                variable=frame_var,
                command=slider_changed,
                bg=panel,
                fg=text,
                troughcolor=raised,
                activebackground=accent,
                highlightthickness=0,
                relief="flat",
                length=420,
                showvalue=False,
            )
            slider.pack(side="left", fill="x", expand=True)
            label(controls_inner, "", 10, "normal", muted, panel).pack(side="left", padx=(8, 0))
            current_label = label(controls_inner, "Ready", 10, "normal", muted, panel)
            current_label.pack(side="left", padx=(10, 0))
            current_path_var.trace_add("write", lambda *_args: current_label.configure(text=current_path_var.get()))

            lines: List[str] = []
            if field_image_source is not None and field_image_file is not None:
                lines += [f"Backdrop: {field_image_file.name}", f"Field: {field_x:.2f}m x {field_y:.2f}m", "Scaling: image pixels mapped across navgrid field size", ""]
            elif field_image_error:
                lines += ["Backdrop unavailable:", f"  {field_image_error}", "Install Pillow with: pip install pillow", ""]
            else:
                lines += ["Backdrop unavailable:", "  Missing BattlecryField26.png next to the script or in the PathPlanner folder", ""]
            if warnings:
                lines += ["Warnings:"] + [f"  - {warning}" for warning in warnings] + [""]
            lines += ["Path sequence:"]
            for index, path_name in enumerate(path_names):
                lines.append(f"{index + 1:02d}. {path_name}")
            lines += ["", "Controls:", "Play/Pause = animate", "Slider = scrub", "Restart = beginning"]
            sidebar.insert("1.0", "\n".join(lines))
            sidebar.configure(state="disabled")

            field.bind("<Configure>", lambda _event: draw_frame())
            dialog.protocol("WM_DELETE_WINDOW", lambda: (playing.set(False), dialog.destroy()))
            draw_frame()
        except Exception as exc:
            messagebox.showerror("Visualize failed", str(exc))

    def bind_update(*variables: Any) -> None:
        for variable in variables:
            variable.trace_add("write", update_status)

    def rebuild_titles() -> None:
        for index, data in enumerate(pass_rows):
            data["title"].configure(text=f"Pass {index + 1}")
            data["subtitle"].configure(text="Rush or sneaky start" if index == 0 else "Starts from the shooting location")
            if index == 0:
                data["start_box"].grid(row=1, column=0, sticky="ew", pady=(12, 0))
                data["route_box"].grid_forget()
                data["delete_button"].grid_remove()
            else:
                data["start_box"].grid_forget()
                data["route_box"].grid(row=3, column=0, sticky="ew", pady=(10, 0))
                data["delete_button"].grid(row=0, column=1, sticky="e")

    def remove_pass(data: Dict[str, Any]) -> None:
        if pass_rows and data is pass_rows[0]:
            messagebox.showinfo("First pass", "Pass 1 cannot be deleted.")
            return
        data["frame"].destroy()
        pass_rows.remove(data)
        rebuild_titles()
        update_status()

    def make_pass() -> Dict[str, Any]:
        frame = card_frame(pass_frame, card)
        frame.pack(fill="x", padx=(0, 8), pady=(0, 12))
        content = tk.Frame(frame, bg=card)
        content.pack(fill="x", padx=14, pady=14)
        content.columnconfigure(0, weight=1)

        head = tk.Frame(content, bg=card)
        head.grid(row=0, column=0, sticky="ew")
        head.columnconfigure(0, weight=1)
        title_group = tk.Frame(head, bg=card)
        title_group.grid(row=0, column=0, sticky="w")
        title = label(title_group, "Pass", 14, "bold", text, card)
        title.pack(anchor="w")
        subtitle = label(title_group, "", 9, "normal", muted, card)
        subtitle.pack(anchor="w", pady=(2, 0))
        delete_button = button(head, "Delete", lambda: remove_pass(data), danger, text, 10)
        delete_button.grid(row=0, column=1, sticky="e")

        start = tk.StringVar(value="rush")
        risky = tk.BooleanVar(value=False)
        greedy = tk.BooleanVar(value=False)
        hub = tk.BooleanVar(value=True)
        route = tk.StringVar(value="Far -> Close")
        localize = tk.BooleanVar(value=True)

        start_box = tk.Frame(content, bg=raised)
        label(start_box, "Start", 10, "bold", text, raised).pack(side="left", padx=(12, 12), pady=10)
        radio(start_box, "Rush", start, "rush", raised).pack(side="left", padx=(0, 12))
        radio(start_box, "Sneaky", start, "sneaky", raised).pack(side="left", padx=(0, 12))

        toggles = tk.Frame(content, bg=card)
        toggles.grid(row=2, column=0, sticky="ew", pady=(12, 0))
        check(toggles, "Risky", risky).pack(side="left", padx=(0, 16))
        check(toggles, "Greedy", greedy).pack(side="left", padx=(0, 16))
        check(toggles, "Hub Sweep", hub).pack(side="left", padx=(0, 16))

        route_box = tk.Frame(content, bg=card)
        label(route_box, "Route", 10, "bold", text, card).pack(side="left", padx=(0, 10))
        combo(route_box, route, ["Far -> Close", "Close -> Far"], 15).pack(side="left", padx=(0, 18))
        check(route_box, "Localize", localize).pack(side="left")

        data = {
            "frame": frame,
            "title": title,
            "subtitle": subtitle,
            "delete_button": delete_button,
            "start_box": start_box,
            "route_box": route_box,
            "start": start,
            "risky": risky,
            "greedy": greedy,
            "hub": hub,
            "route": route,
            "localize": localize,
        }

        bind_update(start, risky, greedy, hub, route, localize)
        pass_rows.append(data)
        rebuild_titles()
        update_status()
        return data

    def clear_final() -> None:
        nonlocal final_pass
        if final_pass:
            final_pass["frame"].destroy()
            final_pass = None
            update_status()

    def add_final(kind: str) -> None:
        nonlocal final_pass
        clear_final()

        frame = card_frame(pass_frame, card)
        frame.pack(fill="x", padx=(0, 8), pady=(0, 12))
        content = tk.Frame(frame, bg=card)
        content.pack(fill="x", padx=14, pady=14)
        content.columnconfigure(0, weight=1)

        head = tk.Frame(content, bg=card)
        head.grid(row=0, column=0, sticky="ew")
        head.columnconfigure(0, weight=1)
        title_group = tk.Frame(head, bg=card)
        title_group.grid(row=0, column=0, sticky="w")
        label(title_group, "Final", 14, "bold", text, card).pack(anchor="w")
        label(title_group, "Ends at a dot or does one last non-returning sweep", 9, "normal", muted, card).pack(anchor="w", pady=(2, 0))
        button(head, "Delete", clear_final, danger, text, 10).grid(row=0, column=1, sticky="e")

        final_type = tk.StringVar(value=kind)
        dot = tk.StringVar(value="center")
        risky = tk.BooleanVar(value=False)
        greedy = tk.BooleanVar(value=False)
        hub = tk.BooleanVar(value=True)
        route = tk.StringVar(value="Far -> Close")

        type_box = tk.Frame(content, bg=raised)
        type_box.grid(row=1, column=0, sticky="ew", pady=(12, 0))
        label(type_box, "Type", 10, "bold", text, raised).pack(side="left", padx=(12, 12), pady=10)
        radio(type_box, "Sweep", final_type, "sweep", raised).pack(side="left", padx=(0, 12))
        radio(type_box, "Dot", final_type, "dot", raised).pack(side="left", padx=(0, 12))

        sweep_box = tk.Frame(content, bg=card)
        dot_box = tk.Frame(content, bg=card)

        check(sweep_box, "Risky", risky).pack(side="left", padx=(0, 16))
        check(sweep_box, "Greedy", greedy).pack(side="left", padx=(0, 16))
        check(sweep_box, "Hub Sweep", hub).pack(side="left", padx=(0, 16))
        label(sweep_box, "Route", 10, "bold", text, card).pack(side="left", padx=(0, 10))
        combo(sweep_box, route, ["Far -> Close", "Close -> Far"], 15).pack(side="left")

        label(dot_box, "Dot", 10, "bold", text, card).pack(side="left", padx=(0, 8))
        combo(dot_box, dot, ["center", "close"], 10).pack(side="left")

        def update_final_visible(*_args: Any) -> None:
            if final_type.get() == "dot":
                sweep_box.grid_forget()
                dot_box.grid(row=2, column=0, sticky="ew", pady=(12, 0))
            else:
                dot_box.grid_forget()
                sweep_box.grid(row=2, column=0, sticky="ew", pady=(12, 0))
            update_status()

        for variable in [final_type, dot, risky, greedy, hub, route]:
            variable.trace_add("write", update_final_visible)

        final_pass = {
            "frame": frame,
            "type": final_type,
            "dot": dot,
            "risky": risky,
            "greedy": greedy,
            "hub": hub,
            "route": route,
        }
        update_final_visible()

    def clear_editor() -> None:
        nonlocal final_pass
        for row in list(pass_rows):
            row["frame"].destroy()
        pass_rows.clear()
        if final_pass:
            final_pass["frame"].destroy()
            final_pass = None

    def apply_pass_spec(row: Dict[str, Any], spec: Dict[str, Any]) -> None:
        row["start"].set(spec.get("start", "rush"))
        row["risky"].set(bool(spec.get("risky", False)))
        row["greedy"].set(bool(spec.get("greedy", False)))
        row["hub"].set(bool(spec.get("hub", True)))
        row["route"].set(spec.get("route", "Far -> Close"))
        row["localize"].set(bool(spec.get("localize", True)))

    def populate_editor(pass_specs: List[Dict[str, Any]], final_spec: Optional[Dict[str, Any]]) -> None:
        clear_editor()
        if not pass_specs:
            pass_specs = [{"start": "rush", "risky": False, "greedy": False, "hub": True, "route": "Far -> Close", "localize": True}]
        for spec in pass_specs:
            row = make_pass()
            apply_pass_spec(row, spec)
        if final_spec:
            add_final(final_spec.get("type", "dot"))
            if final_pass:
                final_pass["type"].set(final_spec.get("type", "dot"))
                if final_spec.get("type") == "dot":
                    final_pass["dot"].set(final_spec.get("dot", "center"))
                else:
                    final_pass["risky"].set(bool(final_spec.get("risky", False)))
                    final_pass["greedy"].set(bool(final_spec.get("greedy", False)))
                    final_pass["hub"].set(bool(final_spec.get("hub", True)))
                    final_pass["route"].set(final_spec.get("route", "Far -> Close"))
        rebuild_titles()
        update_status()

    def settings_dialog() -> None:
        dialog = tk.Toplevel(root)
        dialog.title("Settings")
        dialog.configure(bg=bg)
        dialog.resizable(False, False)
        frame = card_frame(dialog, panel)
        frame.pack(fill="both", expand=True, padx=18, pady=18)
        content = tk.Frame(frame, bg=panel)
        content.pack(fill="both", expand=True, padx=16, pady=16)
        label(content, "Settings", 16, "bold", text, panel).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 12))
        label(content, "PathPlanner folder", 10, "normal", muted, panel).grid(row=1, column=0, columnspan=3, sticky="w")
        entry = tk.Entry(content, textvariable=project_var, width=56, bg=raised, fg=text, insertbackground=text, relief="flat", bd=0, font=("Segoe UI", 10))
        entry.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 12), ipady=8)

        def browse() -> None:
            folder = filedialog.askdirectory(initialdir=str(Path.cwd()))
            if folder:
                project_var.set(folder)

        button(content, "Browse", browse, raised).grid(row=2, column=2, padx=(8, 0), pady=(6, 12))
        button(content, "Close", dialog.destroy, accent).grid(row=3, column=2, sticky="e")

    def save_dialog() -> None:
        try:
            current_commands()
        except Exception as exc:
            messagebox.showerror("Cannot save", str(exc))
            return

        dialog = tk.Toplevel(root)
        dialog.title("Save Auto")
        dialog.configure(bg=bg)
        dialog.resizable(False, False)
        frame = card_frame(dialog, panel)
        frame.pack(fill="both", expand=True, padx=18, pady=18)
        content = tk.Frame(frame, bg=panel)
        content.pack(fill="both", expand=True, padx=16, pady=16)
        label(content, "Save Auto", 16, "bold", text, panel).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 12))
        label(content, "Auto name", 10, "normal", muted, panel).grid(row=1, column=0, columnspan=2, sticky="w")
        name_entry = tk.Entry(content, textvariable=auto_name_var, width=48, bg=raised, fg=text, insertbackground=text, relief="flat", bd=0, font=("Segoe UI", 10))
        name_entry.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 12), ipady=8)
        check(content, "Save as copy", save_copy_var, panel).grid(row=3, column=0, sticky="w")
        check(content, "Overwrite existing", overwrite_var, panel).grid(row=4, column=0, sticky="w", pady=(0, 12))

        def save() -> None:
            name = safe_filename(auto_name_var.get())
            if not name:
                messagebox.showerror("Missing name", "Enter an auto name.")
                return

            try:
                project = resolve_project(Path(project_var.get()))
                commands, _warnings, folder = current_commands(name)
                auto_json = make_auto(commands, folder=folder, reset_odom=True)
                path_mapping: Optional[Dict[str, str]] = None

                if save_copy_var.get():
                    path_mapping = copy_paths_for_auto(
                        project=project,
                        auto_json=auto_json,
                        auto_name=name,
                        copy_folder=name,
                        waypoint_prefix=f"{name}_",
                        overwrite=overwrite_var.get(),
                    )

                written = write_auto(project, name, auto_json, overwrite_var.get())
                print_auto_summary(name, auto_json, path_mapping)
                dialog.destroy()
                messagebox.showinfo("Saved", f"Wrote {written.name}")
                update_status()
            except Exception as exc:
                messagebox.showerror("Save failed", str(exc))

        bar = tk.Frame(content, bg=panel)
        bar.grid(row=5, column=0, columnspan=2, sticky="e")
        button(bar, "Cancel", dialog.destroy, raised).pack(side="left", padx=(0, 8))
        button(bar, "Save", save, success).pack(side="left")
        name_entry.focus_set()

    def load_auto() -> None:
        try:
            project = resolve_project(Path(project_var.get()))
            file = filedialog.askopenfilename(
                initialdir=str(autos_dir(project)),
                title="Load auto",
                filetypes=[("PathPlanner Auto", "*.auto"), ("All files", "*.*")],
            )
        except Exception as exc:
            messagebox.showerror("Load failed", str(exc))
            return

        if not file:
            return

        try:
            data = load_json(Path(file))
            auto_name_var.set(Path(file).stem)
            pass_specs, final_spec = parse_generated_auto(data)
            loaded_custom_auto.set("")
            populate_editor(pass_specs, final_spec)
            messagebox.showinfo("Loaded", f"Loaded {Path(file).stem} into the editor.")
        except Exception:
            try:
                data = load_json(Path(file))
                auto_name_var.set(Path(file).stem)
                paths = collect_path_names(data.get("command", {}))
                named_commands = [
                    c.get("data", {}).get("name")
                    for c in iter_commands(data.get("command", {}))
                    if c.get("type") == "named"
                ]
                loaded_custom_auto.set("Custom auto selected, cannot parse into pass editor.")
                lines = [
                    f"Loaded: {Path(file).stem}",
                    "Custom Auto Selected",
                    "Cannot parse this auto into the pass editor.",
                    "",
                    "Path sequence:",
                ]
                lines += [f"  {index + 1:02d}. {p}" for index, p in enumerate(paths)] or ["  None"]
                lines += ["", "Named commands:"]
                lines += [f"  {index + 1:02d}. {c}" for index, c in enumerate(named_commands)] or ["  None"]
                show_text_window("Loaded Auto", "\n".join(lines))
                update_status()
            except Exception as exc:
                messagebox.showerror("Load failed", str(exc))

    def list_items() -> None:
        try:
            project = resolve_project(Path(project_var.get()))
            lines = [f"Project: {project}", "", "Autos:"]
            lines += [f"  {name}" for name in list_auto_names(project)]
            lines += ["", "Paths:"]
            lines += [f"  {name}" for name in list_path_names(project)]
            show_text_window("Project Files", "\n".join(lines))
        except Exception as exc:
            messagebox.showerror("List failed", str(exc))

    button(header_actions, "Settings", settings_dialog, raised).pack(side="right", padx=(8, 0))
    button(header_actions, "Load Auto", load_auto, raised).pack(side="right", padx=(8, 0))
    button(header_actions, "Preview", show_preview, raised).pack(side="right", padx=(8, 0))
    button(header_actions, "Visualize", show_visualization, raised).pack(side="right", padx=(8, 0))
    button(header_actions, "Save", save_dialog, accent).pack(side="right", padx=(8, 0))

    add_bar = tk.Frame(toolbar_inner, bg=panel)
    add_bar.pack(side="left")
    button(add_bar, "+ Add Pass", make_pass, accent).pack(side="left", padx=(0, 8))
    button(add_bar, "+ Final Sweep", lambda: add_final("sweep"), raised).pack(side="left", padx=(0, 8))
    button(add_bar, "+ Final Dot", lambda: add_final("dot"), raised).pack(side="left", padx=(0, 8))
    button(toolbar_inner, "List", list_items, raised).pack(side="right")

    make_pass()
    update_status()
    root.mainloop()
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

    gui_parser = subparsers.add_parser("gui", help="Open the GUI")
    gui_parser.set_defaults(func=cmd_gui)

    return parser


def main() -> int:
    if len(sys.argv) == 1:
        return launch_gui()

    parser = build_parser()
    args = parser.parse_args()

    try:
        return args.func(args)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
