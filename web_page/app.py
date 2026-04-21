from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from flask import Flask, render_template, request, redirect, url_for, jsonify, send_file, abort, session
import socket
import threading
import time    
import csv   
import random
import struct
import subprocess
import sys
import numpy as np
import logging
from threading import Thread 

import queue 

logging.getLogger('werkzeug').disabled = True #Suppress werkzeug logs

app = Flask(__name__)
app.secret_key = os.environ.get("SONICSOLE_SECRET_KEY", "sonicsole-local-secret")
app.config["TEMPLATES_AUTO_RELOAD"] = True
app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0
app.jinja_env.auto_reload = True
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GROUP_ASSIGNMENT_DISCOVERY_WAIT_SECONDS = float(os.environ.get("SONICSOLE_GROUP_ASSIGNMENT_WAIT_SECONDS", "1.0"))
HARDWARE_STATUS_DISCOVERY_WAIT_SECONDS = float(os.environ.get("SONICSOLE_HARDWARE_STATUS_WAIT_SECONDS", "0.35"))
DISCOVERED_DEVICE_STALE_SECONDS = float(os.environ.get("SONICSOLE_DEVICE_STALE_SECONDS", "5.0"))

GROUP_SLOTS = [
    {
        "id": f"group_{group_number}",
        "label": f"Group {group_number}",
        "number": group_number,
    }
    for group_number in range(1, 6)
]
GROUP_OPTIONS_BY_ID = {group["id"]: group for group in GROUP_SLOTS}
HARDWARE_PING_TIMEOUT_SECONDS = 1.0

PHONE_ACTIVITY_CARDS = [
    {
        "slug": "jump",
        "label": "Jumping",
        "kicker": "Performance",
        "image": "Jumping.png",
        "template": "jump.html",
        "phone_activity": "jump",
    },
    {
        "slug": "balance",
        "label": "Balancing",
        "kicker": "Stability",
        "image": "Balancing.png",
        "template": "balance.html",
        "phone_activity": "balance",
    },
    {
        "slug": "reaction",
        "label": "Reaction",
        "kicker": "Timing",
        "image": "Reaction.png",
        "template": "reaction.html",
        "phone_activity": "reaction",
    },
    {
        "slug": "precision",
        "label": "Precision",
        "kicker": "Control",
        "image": "Pressure.png",
        "template": "precision.html",
        "phone_activity": "precision",
    },
]
PHONE_ACTIVITY_CONFIG = {
    activity["slug"]: activity
    for activity in PHONE_ACTIVITY_CARDS
}
PHONE_ACTIVITY_CARDS_NO_WALK = [
    activity
    for activity in PHONE_ACTIVITY_CARDS
    if activity["slug"] != "walk"
]

LEADERBOARD_CONFIG = {
    "balance": {
        "file": "SonicSoleBalance.txt",
        "route": "b_scoreboard",
        "value_field": "time",
        "sort_reverse": True,
    },
    "jump": {
        "file": "SonicSoleJump.txt",
        "route": "j_scoreboard",
        "value_field": "last_jump_height",
        "sort_reverse": True,
    },
    "reaction": {
        "file": "SonicSoleReaction.txt",
        "route": "r_scoreboard",
        "value_field": "reaction_time",
        "sort_reverse": False,
    },
    "precision": {
        "file": "SonicSolePrecision.txt",
        "route": "p_scoreboard",
        "value_field": "percent_error",
        "sort_reverse": False,
    },
    "walk": {
        "file": "SonicSoleWalk.txt",
        "route": "w_scoreboard",
        "value_field": "forefoot_dist",
        "sort_reverse": True,
    },
}

GROUP_HISTORY_META = {
    "jump": {
        "label": "Jumping",
        "decimals": 2,
        "unit": "m",
    },
    "balance": {
        "label": "Balancing",
        "decimals": 3,
        "unit": "s",
    },
    "reaction": {
        "label": "Reaction",
        "decimals": 3,
        "unit": "s",
    },
    "precision": {
        "label": "Precision",
        "decimals": 1,
        "unit": "%",
    },
    "walk": {
        "label": "Forefoot Walk",
        "decimals": 2,
        "unit": "m",
    },
}

DUMMY_LEADERBOARD_ROWS = {
    "balance": [
        ["Group 1", "48.3"],
        ["Group 2", "42.6"],
        ["Group 3", "53.8"],
        ["Group 4", "37.1"],
        ["Group 5", "45.2"],
    ],
    "jump": [
        ["Group 1", "0.42"],
        ["Group 2", "0.55"],
        ["Group 3", "0.48"],
        ["Group 4", "0.51"],
        ["Group 5", "0.46"],
    ],
    "reaction": [
        ["Group 1", "3.24"],
        ["Group 2", "2.88"],
        ["Group 3", "3.57"],
        ["Group 4", "2.94"],
        ["Group 5", "3.12"],
    ],
    "precision": [
        ["Group 1", "4.8"],
        ["Group 2", "6.1"],
        ["Group 3", "3.9"],
        ["Group 4", "5.2"],
        ["Group 5", "4.4"],
    ],
    "walk": [
        ["Group 1", "2.8"],
        ["Group 2", "3.1"],
        ["Group 3", "2.6"],
        ["Group 4", "3.4"],
        ["Group 5", "2.9"],
    ],
}

PC_SCOREBOARD_KEYS = ("jump", "balance", "reaction", "precision")
PC_SCOREBOARD_META = {
    "jump": {
        "label": "Jumping",
        "title": "Jumping Scoreboard",
        "summary": "Best jump height from each group.",
    },
    "balance": {
        "label": "Balancing",
        "title": "Balancing Scoreboard",
        "summary": "Longest balance hold from each group.",
    },
    "reaction": {
        "label": "Reaction",
        "title": "Reaction Scoreboard",
        "summary": "Fastest reaction time from each group.",
    },
    "precision": {
        "label": "Precision",
        "title": "Precision Scoreboard",
        "summary": "Lowest precision error from each group.",
    },
}


def get_leaderboard_config(board_key):
    config = LEADERBOARD_CONFIG.get(board_key)
    if config is None:
        abort(404)
    return config


def get_leaderboard_path(board_key):
    filename = get_leaderboard_config(board_key)["file"]
    return os.path.join(PROJECT_ROOT, filename)


def get_sample_group_history_path(group):
    resolved_group = resolve_group_slot(group)
    if resolved_group is None:
        return ""

    return os.path.join(
        PROJECT_ROOT,
        "web_page",
        "static",
        "sample_group_history",
        f"{resolved_group['id']}.json",
    )


def parse_score(value, allow_blank=False):
    text = "" if value is None else str(value).strip()
    if not text:
        return None if allow_blank else None
    try:
        return float(text)
    except ValueError:
        return None


def get_display_name_from_entry(raw_name):
    name = (raw_name or "").strip()
    split_name = name.rsplit("_", 1)
    if len(split_name) == 2 and split_name[1] in {"0", "1"}:
        return split_name[0].strip()
    return name


def normalize_group_name(name):
    return "".join(character for character in str(name or "").lower() if character.isalnum())


def entry_matches_group(raw_name, target_group_name):
    return normalize_group_name(get_display_name_from_entry(raw_name)) == normalize_group_name(target_group_name)


def resolve_group_slot(group):
    if isinstance(group, str):
        return GROUP_OPTIONS_BY_ID.get(group)
    if group is None:
        return None
    if "number" in group:
        return group

    group_id = group.get("id") if isinstance(group, dict) else None
    if group_id:
        return GROUP_OPTIONS_BY_ID.get(group_id)

    group_slug = group.get("slug") if isinstance(group, dict) else None
    if group_slug:
        return get_group_from_slug(group_slug)

    return None


def get_group_phone_slug(group):
    resolved_group = resolve_group_slot(group)
    if resolved_group is None:
        return ""
    return f"group{resolved_group['number']}"


def get_group_from_slug(group_slug):
    normalized_slug = "".join(character for character in str(group_slug or "").lower() if character.isalnum())
    if normalized_slug.startswith("group"):
        normalized_slug = normalized_slug[5:]
    if not normalized_slug.isdigit():
        return None
    return GROUP_OPTIONS_BY_ID.get(f"group_{int(normalized_slug)}")


def build_group_option(group, assigned_ip=None):
    return {
        "id": group["id"],
        "label": group["label"],
        "slug": get_group_phone_slug(group),
        "ip": (assigned_ip if assigned_ip is not None else get_group_assignment_ip(group["id"])).strip(),
    }


def get_group_options():
    return [build_group_option(group) for group in GROUP_SLOTS]


def get_default_sensor_check_group():
    group_options = get_group_options()
    for group in group_options:
        if group["ip"]:
            return group
    return group_options[0] if group_options else None


def get_group_assignment_ip(group_id):
    with group_assignments_lock:
        return (group_device_assignments.get(group_id) or "").strip()


def get_group_assignment_map():
    with group_assignments_lock:
        return {
            assigned_group_id: (assigned_ip or "").strip()
            for assigned_group_id, assigned_ip in group_device_assignments.items()
            if (assigned_ip or "").strip()
        }


def get_group_id_for_assigned_ip(device_ip, exclude_group_id=None):
    normalized_ip = (device_ip or "").strip()
    if not normalized_ip:
        return None

    with group_assignments_lock:
        for assigned_group_id, assigned_ip in group_device_assignments.items():
            if assigned_group_id == exclude_group_id:
                continue
            if (assigned_ip or "").strip() == normalized_ip:
                return assigned_group_id
    return None


def assign_group_device_ip(group_id, device_ip):
    normalized_ip = (device_ip or "").strip()
    group = GROUP_OPTIONS_BY_ID.get(group_id)
    if group is None:
        return None, None

    with group_assignments_lock:
        if normalized_ip:
            for assigned_group_id, assigned_ip in group_device_assignments.items():
                if assigned_group_id == group_id:
                    continue
                if (assigned_ip or "").strip() == normalized_ip:
                    conflicting_group = GROUP_OPTIONS_BY_ID.get(assigned_group_id)
                    return (
                        None,
                        build_group_option(conflicting_group, assigned_ip=(assigned_ip or "").strip()),
                    )
            group_device_assignments[group_id] = normalized_ip
        else:
            group_device_assignments.pop(group_id, None)

    return build_group_option(group, normalized_ip), None


def prune_discovered_devices_locked(current_time=None):
    now = time.time() if current_time is None else current_time
    cutoff = now - DISCOVERED_DEVICE_STALE_SECONDS

    stale_ips = [
        device_ip
        for device_ip, device in discovered_devices.items()
        if device.get("last_seen", 0.0) < cutoff
    ]
    for device_ip in stale_ips:
        discovered_devices.pop(device_ip, None)


def record_discovered_device(device_ip):
    if not device_ip:
        return

    now = time.time()
    with discovered_devices_lock:
        prune_discovered_devices_locked(now)
        entry = discovered_devices.get(
            device_ip,
            {
                "ip": device_ip,
                "first_seen": now,
                "last_seen": now,
                "packet_count": 0,
            },
        )
        entry["last_seen"] = now
        entry["packet_count"] = int(entry.get("packet_count", 0)) + 1
        discovered_devices[device_ip] = entry


def get_discovered_devices():
    with discovered_devices_lock:
        prune_discovered_devices_locked()
        return sorted(
            [
                {
                    "ip": device["ip"],
                    "first_seen": device["first_seen"],
                    "last_seen": device["last_seen"],
                    "packet_count": device["packet_count"],
                }
                for device in discovered_devices.values()
            ],
            key=lambda device: device["last_seen"],
            reverse=True,
        )


def get_latest_discovered_device():
    devices = get_discovered_devices()
    return devices[0] if devices else None


def get_assignable_discovered_devices(group_id):
    current_group_ip = get_group_assignment_ip(group_id)
    assigned_ips = set(get_group_assignment_map().values())
    if current_group_ip:
        assigned_ips.discard(current_group_ip)

    assignable_devices = []
    for device in get_discovered_devices():
        device_ip = device["ip"]
        if device_ip in assigned_ips:
            continue
        assignable_devices.append(
            {
                **device,
                "selected": device_ip == current_group_ip,
            }
        )
    return assignable_devices


def wait_for_discovered_device(timeout_seconds):
    deadline = time.time() + max(0.0, timeout_seconds)
    while True:
        latest_device = get_latest_discovered_device()
        if latest_device is not None:
            return latest_device
        if time.time() >= deadline:
            return None
        time.sleep(0.05)


def get_selected_group():
    group_id = session.get("selected_group_id")
    if not group_id:
        return None

    group = GROUP_OPTIONS_BY_ID.get(group_id)
    if group is None:
        session.pop("selected_group_id", None)
        return None
    return build_group_option(group)


def set_selected_group(group_id, activate_device=False):
    group = GROUP_OPTIONS_BY_ID.get(group_id)
    if group is None:
        session.pop("selected_group_id", None)
        if activate_device:
            set_active_device_ip(None)
        return None

    session["selected_group_id"] = group_id
    selected_group = build_group_option(group)
    if activate_device:
        set_active_device_ip((selected_group or {}).get("ip"))
    return selected_group


def clear_selected_group():
    session.pop("selected_group_id", None)
    set_active_device_ip(None)


def set_active_device_ip(device_ip):
    global active_device_ip
    with active_device_ip_lock:
        active_device_ip = device_ip.strip() if device_ip else None


def get_active_device_ip():
    with active_device_ip_lock:
        return active_device_ip


def activate_selected_group():
    selected_group = get_selected_group()
    if selected_group is None:
        return None

    device_ip = (selected_group.get("ip") or "").strip()
    if not device_ip:
        return None

    set_active_device_ip(device_ip)
    return selected_group


def require_selected_group():
    selected_group = get_selected_group()
    if selected_group is None:
        return None, (
            jsonify(
                {
                    "status": "error",
                    "message": "Please select a group before starting the activity.",
                }
            ),
            400,
        )

    if not (selected_group.get("ip") or "").strip():
        start_combined_data_thread()
        live_device = wait_for_discovered_device(GROUP_ASSIGNMENT_DISCOVERY_WAIT_SECONDS)
        live_suffix = f" Live device: {live_device['ip']}." if live_device is not None else ""
        return None, (
            jsonify(
                {
                    "status": "error",
                    "message": f"{selected_group['label']} does not have a device assigned yet. Assign the live device from Hardware Monitor first.{live_suffix}",
                }
            ),
            400,
        )

    selected_group = activate_selected_group()
    if selected_group is not None:
        return selected_group, None

    return None, (
        jsonify(
            {
                "status": "error",
                "message": "Unable to activate the selected group.",
            }
        ),
        400,
    )


def get_group_name_from_request(*preferred_fields):
    field_names = []
    seen = set()

    for field_name in list(preferred_fields) + ["group_name", "name"]:
        if field_name and field_name not in seen:
            field_names.append(field_name)
            seen.add(field_name)

    for field_name in field_names:
        value = request.form.get(field_name, "").strip()
        if value:
            return value

    selected_group = get_selected_group()
    if selected_group is not None:
        return selected_group["label"]

    for first_key, last_key in (("first_name", "last_name"), ("first_name2", "last_name2")):
        first = request.form.get(first_key, "").strip()
        last = request.form.get(last_key, "").strip()
        combined = " ".join(part for part in (first, last) if part).strip()
        if combined:
            return combined

    return ""


def write_leaderboard_rows(board_key, rows):
    with open(get_leaderboard_path(board_key), "w", newline="") as file_obj:
        writer = csv.writer(file_obj)
        writer.writerows(rows)


def append_leaderboard_row(board_key, name, value):
    with open(get_leaderboard_path(board_key), "a", newline="") as file_obj:
        writer = csv.writer(file_obj)
        writer.writerow([name, value])


def read_leaderboard_rows(board_key):
    path = get_leaderboard_path(board_key)
    if not os.path.exists(path):
        write_leaderboard_rows(board_key, DUMMY_LEADERBOARD_ROWS[board_key])

    rows = []
    with open(path, "r", newline="") as file_obj:
        reader = csv.reader(file_obj)
        for row in reader:
            if len(row) >= 2:
                rows.append([row[0].strip(), row[1].strip()])
    return rows


def load_balance_leaderboard():
    best_scores = {}
    for raw_name, raw_value in read_leaderboard_rows("balance"):
        score = parse_score(raw_value)
        if score is None or not raw_name:
            continue

        split_name = raw_name.rsplit("_", 1)
        display_name = raw_name.strip()
        if len(split_name) == 2 and split_name[1] in {"0", "1"}:
            display_name = split_name[0].strip()

        current_best = best_scores.get(display_name)
        if current_best is None or score > current_best:
            best_scores[display_name] = score

    entries = [
        {
            "entry_key": name,
            "name": name,
            "best_time": score,
        }
        for name, score in best_scores.items()
    ]
    entries.sort(key=lambda entry: entry["best_time"], reverse=True)
    return entries


def serialize_balance_leaderboard(entries):
    rows = []
    for entry in entries:
        name = entry["name"].strip()
        best_time = parse_score(entry.get("best_time"), allow_blank=True)
        if not name or best_time is None:
            continue
        rows.append([name, f"{best_time:.4f}"])
    return rows


def load_metric_leaderboard(board_key):
    config = get_leaderboard_config(board_key)
    value_field = config["value_field"]
    best_scores = {}

    for raw_name, raw_value in read_leaderboard_rows(board_key):
        score = parse_score(raw_value)
        name = raw_name.strip()
        if score is None or not name:
            continue

        if name not in best_scores:
            best_scores[name] = score
            continue

        if config["sort_reverse"]:
            best_scores[name] = max(best_scores[name], score)
        else:
            best_scores[name] = min(best_scores[name], score)

    entries = [{"entry_key": name, "name": name, value_field: score} for name, score in best_scores.items()]
    entries.sort(key=lambda entry: entry[value_field], reverse=config["sort_reverse"])
    return entries


def load_group_best_performance(board_key):
    if board_key not in PC_SCOREBOARD_KEYS:
        abort(404)

    config = get_leaderboard_config(board_key)
    best_scores_by_group_id = {}

    for raw_name, raw_value in read_leaderboard_rows(board_key):
        score = parse_score(raw_value, allow_blank=True)
        if score is None:
            continue

        matched_group = None
        for group in GROUP_SLOTS:
            if entry_matches_group(raw_name, group["label"]):
                matched_group = group
                break

        if matched_group is None:
            continue

        current_best = best_scores_by_group_id.get(matched_group["id"])
        if current_best is None:
            best_scores_by_group_id[matched_group["id"]] = score
        elif config["sort_reverse"]:
            best_scores_by_group_id[matched_group["id"]] = max(current_best, score)
        else:
            best_scores_by_group_id[matched_group["id"]] = min(current_best, score)

    rows = []
    for group in GROUP_SLOTS:
        score = best_scores_by_group_id.get(group["id"])
        rows.append(
            {
                "group_id": group["id"],
                "group_label": group["label"],
                "group_number": group["number"],
                "score": score,
                "formatted_score": format_group_history_value(board_key, score) if score is not None else "--",
                "has_score": score is not None,
            }
        )

    return rows


def serialize_metric_leaderboard(board_key, entries):
    value_field = get_leaderboard_config(board_key)["value_field"]
    rows = []
    for entry in entries:
        name = entry["name"].strip()
        value = entry.get(value_field)
        if not name or value is None:
            continue
        rows.append([name, f"{value:.4f}"])
    return rows


def format_group_history_value(board_key, score):
    meta = GROUP_HISTORY_META[board_key]
    formatted_value = f"{score:.{meta['decimals']}f}"
    if meta["unit"] == "%":
        return f"{formatted_value}%"
    return f"{formatted_value} {meta['unit']}"


def history_has_attempts(history_sections):
    return any((section.get("attempt_count") or 0) > 0 for section in history_sections)


def load_sample_group_history(group):
    sample_path = get_sample_group_history_path(group)
    if not sample_path or not os.path.exists(sample_path):
        return None

    with open(sample_path, "r", encoding="utf-8") as file_obj:
        payload = json.load(file_obj)

    if not isinstance(payload, dict):
        return None
    if not isinstance(payload.get("history"), list):
        return None
    return payload


def load_group_history(group_name):
    history_sections = []

    for board_key, meta in GROUP_HISTORY_META.items():
        attempts = []
        for raw_name, raw_value in reversed(read_leaderboard_rows(board_key)):
            if not entry_matches_group(raw_name, group_name):
                continue

            score = parse_score(raw_value, allow_blank=True)
            if score is None:
                continue

            attempts.append(
                {
                    "value": score,
                    "formatted_value": format_group_history_value(board_key, score),
                }
            )

        best_attempt = None
        if attempts:
            if get_leaderboard_config(board_key)["sort_reverse"]:
                best_attempt = max(attempts, key=lambda attempt: attempt["value"])
            else:
                best_attempt = min(attempts, key=lambda attempt: attempt["value"])

        history_sections.append(
            {
                "board_key": board_key,
                "label": meta["label"],
                "attempt_count": len(attempts),
                "attempts": attempts,
                "best_value": (best_attempt or {}).get("value"),
                "best_formatted_value": (best_attempt or {}).get("formatted_value"),
            }
        )

    return history_sections


def redirect_to_leaderboard(board_key):
    route_name = get_leaderboard_config(board_key)["route"]
    if request.args.get("embed") == "1" or request.form.get("embed") == "1":
        return redirect(url_for(route_name, embed=1))
    return redirect(url_for(route_name))


def build_phone_group_links(group):
    group_slug = get_group_phone_slug(group)
    return {
        "home": url_for("phone_group_home", group_slug=group_slug),
        "setup": url_for("phone_group_setup", group_slug=group_slug),
        "jump": url_for("phone_group_activity", group_slug=group_slug, activity_slug="jump"),
        "balance": url_for("phone_group_activity", group_slug=group_slug, activity_slug="balance"),
        "reaction": url_for("phone_group_activity", group_slug=group_slug, activity_slug="reaction"),
        "precision": url_for("phone_group_activity", group_slug=group_slug, activity_slug="precision"),
    }


def render_phone_group_template(group_slug, template_name, phone_activity, **context):
    group = get_group_from_slug(group_slug)
    if group is None:
        abort(404)

    selected_group = set_selected_group(group["id"])
    render_context = {
        "group_selection_locked": True,
        "hide_group_selector_ui": True,
        "phone_activity": phone_activity,
        "phone_group": selected_group,
        "phone_group_links": build_phone_group_links(group),
    }
    render_context.update(context)
    return render_template(template_name, **render_context)


def render_phone_group_home_page(group_slug, activity_cards=None):
    group = get_group_from_slug(group_slug)
    if group is None:
        abort(404)

    selected_group = set_selected_group(group["id"])
    return render_template(
        'phone_home.html',
        group_selection_locked=True,
        phone_activity="home",
        phone_group=selected_group,
        phone_group_links=build_phone_group_links(group),
        phone_activity_cards=activity_cards or PHONE_ACTIVITY_CARDS,
    )


def render_phone_group_setup_page(group_slug):
    group = get_group_from_slug(group_slug)
    if group is None:
        abort(404)

    start_combined_data_thread()
    selected_group = set_selected_group(group["id"])
    return render_template(
        "assemblyInstructions.html",
        group_selection_locked=True,
        hide_group_selector_ui=True,
        phone_group=selected_group,
        phone_group_links=build_phone_group_links(group),
        sensor_check_default_group=selected_group,
    )


@app.context_processor
def inject_group_selector_context():
    return {
        "group_options": get_group_options(),
        "selected_group": get_selected_group(),
    }


def build_hardware_ping_command(device_ip):
    if sys.platform == "darwin":
        return ["ping", "-c", "1", "-W", "1000", device_ip]
    return ["ping", "-c", "1", "-W", "1", device_ip]


def probe_hardware_device(group):
    resolved_group = build_group_option(group)
    device_ip = (resolved_group.get("ip") or "").strip()
    connected = False
    checked_at = time.time()
    probe_error = None
    assigned = bool(device_ip)

    discovered_ips = {device["ip"] for device in get_discovered_devices()}
    if device_ip and device_ip in discovered_ips:
        connected = True

    if device_ip and not connected:
        try:
            result = subprocess.run(
                build_hardware_ping_command(device_ip),
                capture_output=True,
                text=True,
                check=False,
                timeout=HARDWARE_PING_TIMEOUT_SECONDS + 1.0,
            )
            connected = result.returncode == 0
            if not connected:
                ping_output = f"{result.stdout}\n{result.stderr}".lower()
                if "operation not permitted" in ping_output or "permission denied" in ping_output:
                    probe_error = "permissions"
        except (OSError, subprocess.SubprocessError):
            probe_error = "unavailable"

    return {
        "id": resolved_group["id"],
        "label": resolved_group["label"],
        "ip": device_ip,
        "connected": connected,
        "probe_error": probe_error,
        "checked_at": checked_at,
        "assigned": assigned,
    }


#UDP_IP = "127.0.0.1" #accept data from localhost
UDP_IP = "0.0.0.0" # accept connections on any available network interface of the server
UDP_PORT = 21000
DISCOVERY_PORT = int(os.environ.get("SONICSOLE_DISCOVERY_PORT", "21001"))
DISCOVERY_REQUEST = b"SONICSOLE_DISCOVER_SERVER_V1"
DISCOVERY_RESPONSE = b"SONICSOLE_SERVER_V1"
DISCOVERY_BUFFER_SIZE = 256
bufferSize = 1024

ACTIVITY_COMMAND_PORT = int(os.environ.get("SONICSOLE_ACTIVITY_PORT", "21010"))
ACTIVITY_COMMAND_TIMEOUT_SECONDS = float(os.environ.get("SONICSOLE_ACTIVITY_TIMEOUT", "180.0"))
ACTIVITY_RESPONSE_BUFFER_SIZE = 1024

udp_queue = queue.Queue(maxsize=1) #hold latest packet only

received_heel_data = "0"
received_fore_data = "0"
ax = 0.0 #m/s^2
ay = 0.0
az = 0.0
qx = 0.0
qy = 0.0
qz = 0.0
qw = 1.0
imu_orientation_mode = "identity"
g = 9.81 #m/s^2
received_vertical_raw = "0.0" 

airtime=0
jump_metrics_ready = False
last_jump_height = 0.0
last_airtime = 0.0

jump_vertical_buffer = []
jump_collecting = False
jump_lock = threading.Lock()
balance_state_lock = threading.Lock()
activity_session_lock = threading.Lock()
active_device_ip_lock = threading.Lock()
discovery_listener_lock = threading.Lock()
discovered_devices_lock = threading.Lock()
group_assignments_lock = threading.Lock()
sensor_payloads_lock = threading.Lock()
balance_session_id = 0
recording_time = False
active_device_ip = None
discovery_listener_started = False
discovered_devices = {}
latest_sensor_payloads = {}
group_device_assignments = {
    group["id"]: os.environ.get(f"SONICSOLE_GROUP_{group['number']}_IP", "").strip()
    for group in GROUP_SLOTS
    if os.environ.get(f"SONICSOLE_GROUP_{group['number']}_IP", "").strip()
}
activity_session_ids = {
    "balance": 0,
    "jump": 0,
    "reaction": 0,
    "precision": 0,
    "forefoot": 0,
}

# heel_list = [0 for _ in range(100)]
totalTime = "0"

R_heel = 0
G_heel = 255
R_fore = 0
G_fore = 255

submitted_name = "User1"

submitted_name2 = "User1"

forefoot_status = "waiting"
forefoot_dist = 0
forefoot_elapsed_time = 0

PRECISION_FORCE_MAX = 2000
PRECISION_CAPTURE_SECONDS = 5
PRECISION_TARGET_PERCENTS = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95]

precision_trainer_state = {
    'status': 'idle',
    'max_force': PRECISION_FORCE_MAX,
    'target_percent': 0,
    'target_force': 0,
    'current_force': 0,
    'current_percent': 0,
    'measured_force': 0,
    'measured_percent': 0,
    'error_percent': 0
}


def discovery_listener():
    global discovery_listener_started

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((UDP_IP, DISCOVERY_PORT))
    except OSError as e:
        print(f"[Discovery] Could not bind socket: {e}")
        sock.close()
        with discovery_listener_lock:
            discovery_listener_started = False
        return

    print(f"[Discovery] Listening on UDP {DISCOVERY_PORT}")
    while True:
        try:
            data, addr = sock.recvfrom(DISCOVERY_BUFFER_SIZE)
            if data.strip() != DISCOVERY_REQUEST:
                continue

            sock.sendto(DISCOVERY_RESPONSE, addr)
        except Exception as e:
            print(f"[Discovery] Error: {e}")
            time.sleep(0.1)


def start_discovery_listener():
    global discovery_listener_started

    with discovery_listener_lock:
        if discovery_listener_started:
            return

        discovery_listener_started = True
        thread = threading.Thread(target=discovery_listener, daemon=True)
        thread.start()
precision_error = 0
precision_start_time = None

vertical_raw_data_UDP = []
combined_data_thread = None
combined_data_running = False

reaction_data = {
    "status": "idle",
    "reaction_time": 0.0
}
reaction_time = "0"

threshold_fore = 350
threshold_heel = 350
dt = 0.009 #approx time (s) between samples from udp 

# Audio cues are produced on the RPi (see SonicSole_Audio / ReactionActivity
# in the C++ source). The webapp is headless w.r.t. sound.


def create_balance_session():
    return create_activity_session("balance")


def get_balance_session_id():
    return get_activity_session_id("balance")


def cancel_balance_session():
    global totalTime, recording_time, received_heel_data, received_fore_data
    was_recording = recording_time
    cancel_activity_session("balance")
    recording_time = False
    totalTime = "0"
    received_heel_data = "0"
    received_fore_data = "0"
    stop_combined_data_thread()
    try:
        set_balance_status("idle")
    except NameError:
        pass
    if was_recording:
        cancel_device_ip = resolve_activity_device_ip()
        if cancel_device_ip:
            threading.Thread(
                target=_send_activity_command_fire_and_forget,
                args=(cancel_device_ip, "CANCEL balance"),
                daemon=True,
            ).start()


def _send_activity_command_fire_and_forget(device_ip, command):
    try:
        send_rpi_activity_command(device_ip, command, timeout_seconds=2.0)
    except ActivityCommandError:
        pass
    except Exception:
        pass


def create_activity_session(activity_name):
    with activity_session_lock:
        activity_session_ids[activity_name] += 1
        return activity_session_ids[activity_name]


def get_activity_session_id(activity_name):
    with activity_session_lock:
        return activity_session_ids[activity_name]


def cancel_activity_session(activity_name):
    with activity_session_lock:
        activity_session_ids[activity_name] += 1
        return activity_session_ids[activity_name]


def is_activity_session_active(activity_name, session_id):
    return session_id == get_activity_session_id(activity_name)


def clear_udp_queue():
    while True:
        try:
            udp_queue.get_nowait()
        except queue.Empty:
            break


def reset_jump_state(cancel_session=False):
    global jump_metrics_ready, last_jump_height, last_airtime, jump_collecting, jump_vertical_buffer
    if cancel_session:
        cancel_activity_session("jump")
    jump_metrics_ready = False
    last_jump_height = 0.0
    last_airtime = 0.0
    jump_collecting = False
    with jump_lock:
        jump_vertical_buffer = []


def reset_reaction_state(cancel_session=False):
    global reaction_data, reaction_time
    was_active = bool(reaction_data) and reaction_data.get("status") == "timing"
    if cancel_session:
        cancel_activity_session("reaction")
    reaction_data = {
        "status": "idle",
        "reaction_time": 0.0
    }
    reaction_time = "0"
    if cancel_session and was_active:
        device_ip = resolve_activity_device_ip()
        if device_ip:
            threading.Thread(
                target=_send_activity_command_fire_and_forget,
                args=(device_ip, "CANCEL reaction"),
                daemon=True,
            ).start()


def reset_precision_trainer_state(cancel_session=False):
    global precision_trainer_state, precision_error, precision_start_time
    if cancel_session:
        cancel_activity_session("precision")
    precision_start_time = None
    precision_trainer_state = {
        'status': 'idle',
        'max_force': PRECISION_FORCE_MAX,
        'target_percent': 0,
        'target_force': 0,
        'current_force': 0,
        'current_percent': 0,
        'measured_force': 0,
        'measured_percent': 0,
        'error_percent': 0
    }
    precision_error = 0


def get_precision_force_value(raw_value):
    try:
        pressure_value = int(float(raw_value))
    except (TypeError, ValueError):
        pressure_value = 0
    return max(0, min(PRECISION_FORCE_MAX, pressure_value))


def reset_forefoot_state(cancel_session=False):
    global forefoot_status, forefoot_dist, forefoot_elapsed_time
    if cancel_session:
        cancel_activity_session("forefoot")
    forefoot_status = "waiting"
    forefoot_dist = 0
    forefoot_elapsed_time = 0


def stop_all_activities():
    global R_heel, G_heel, R_fore, G_fore
    cancel_balance_session()
    reset_jump_state(cancel_session=True)
    reset_reaction_state(cancel_session=True)
    reset_precision_trainer_state(cancel_session=True)
    reset_forefoot_state(cancel_session=True)
    clear_udp_queue()
    R_heel = 0
    G_heel = 255
    R_fore = 0
    G_fore = 255


class ActivityCommandError(Exception):
    def __init__(self, reason, detail=""):
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail


def send_rpi_activity_command(device_ip, command, timeout_seconds=ACTIVITY_COMMAND_TIMEOUT_SECONDS):
    """Send a command to the RPi activity listener and block for the reply.

    Parses responses of the form "OK <activity> <score> [reason]" or
    "ERR <activity> <reason>" and returns a dict describing the outcome:

        {"success": bool, "activity": str, "score": float|None, "reason": str}

    Raises ActivityCommandError on network or protocol failures.
    """
    if not device_ip:
        raise ActivityCommandError("no_device", "no RPi device ip available")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.settimeout(max(0.5, timeout_seconds))
        try:
            sock.sendto(command.encode("utf-8"), (device_ip, ACTIVITY_COMMAND_PORT))
        except OSError as send_error:
            raise ActivityCommandError("send_failed", str(send_error)) from send_error
        try:
            payload, _ = sock.recvfrom(ACTIVITY_RESPONSE_BUFFER_SIZE)
        except socket.timeout as timeout_error:
            raise ActivityCommandError("timeout", "no reply from RPi") from timeout_error
        except OSError as recv_error:
            raise ActivityCommandError("recv_failed", str(recv_error)) from recv_error
    finally:
        sock.close()

    response_text = payload.decode("utf-8", errors="replace").strip()
    if not response_text:
        raise ActivityCommandError("empty_reply", "RPi returned empty response")

    tokens = response_text.split()
    status = tokens[0].upper()
    activity_name = tokens[1] if len(tokens) >= 2 else ""

    if status == "OK":
        score_value = None
        if len(tokens) >= 3:
            try:
                score_value = float(tokens[2])
            except ValueError as parse_error:
                raise ActivityCommandError("bad_score", response_text) from parse_error
        reason = " ".join(tokens[3:]) if len(tokens) > 3 else ""
        return {
            "success": True,
            "activity": activity_name,
            "score": score_value,
            "reason": reason,
        }

    if status == "ERR":
        reason = " ".join(tokens[2:]) if len(tokens) > 2 else "unknown"
        return {
            "success": False,
            "activity": activity_name,
            "score": None,
            "reason": reason,
        }

    raise ActivityCommandError("bad_response", response_text)


def resolve_activity_device_ip():
    device_ip = get_active_device_ip()
    if device_ip:
        return device_ip
    selected_group = get_selected_group()
    if selected_group and selected_group.get("ip"):
        return selected_group["ip"]
    return None


def normalize_quaternion(x_value, y_value, z_value, w_value):
    components = np.array([x_value, y_value, z_value, w_value], dtype=np.float64)
    if not np.isfinite(components).all():
        return 0.0, 0.0, 0.0, 1.0

    magnitude = np.linalg.norm(components)
    if magnitude < 1e-8:
        return 0.0, 0.0, 0.0, 1.0

    normalized = components / magnitude
    return tuple(float(component) for component in normalized)


def euler_to_quaternion(roll_radians, pitch_radians, yaw_radians=0.0):
    half_roll = roll_radians * 0.5
    half_pitch = pitch_radians * 0.5
    half_yaw = yaw_radians * 0.5

    sin_roll, cos_roll = math.sin(half_roll), math.cos(half_roll)
    sin_pitch, cos_pitch = math.sin(half_pitch), math.cos(half_pitch)
    sin_yaw, cos_yaw = math.sin(half_yaw), math.cos(half_yaw)

    x_value = (sin_roll * cos_pitch * cos_yaw) - (cos_roll * sin_pitch * sin_yaw)
    y_value = (cos_roll * sin_pitch * cos_yaw) + (sin_roll * cos_pitch * sin_yaw)
    z_value = (cos_roll * cos_pitch * sin_yaw) - (sin_roll * sin_pitch * cos_yaw)
    w_value = (cos_roll * cos_pitch * cos_yaw) + (sin_roll * sin_pitch * sin_yaw)

    return normalize_quaternion(x_value, y_value, z_value, w_value)


def estimate_quaternion_from_acceleration(ax_value, ay_value, az_value):
    magnitude = math.sqrt((ax_value * ax_value) + (ay_value * ay_value) + (az_value * az_value))
    if magnitude < 1e-6:
        return 0.0, 0.0, 0.0, 1.0

    normalized_x = ax_value / magnitude
    normalized_y = ay_value / magnitude
    normalized_z = az_value / magnitude

    safe_z = normalized_z
    if abs(safe_z) < 1e-6:
        safe_z = math.copysign(1e-6, safe_z if safe_z != 0 else 1.0)

    roll_radians = math.atan2(normalized_y, safe_z)
    pitch_radians = math.atan2(-normalized_x, math.sqrt((normalized_y * normalized_y) + (normalized_z * normalized_z)))
    return euler_to_quaternion(roll_radians, pitch_radians, 0.0)

# cumulative trapezoid without scipy because the pi cannot download it
def cumulative_trapezoid_manual(y, dx=1.0, initial=0):
    y = np.asarray(y, dtype=np.float64)
    n = y.shape[0]

    if n < 2:
        return np.array([initial]) if initial is not None else np.array([])

    # Compute trapezoid integration (without initial)
    result = np.empty(n - 1, dtype=np.float64)
    for i in range(n - 1):
        result[i] = 0.5 * (y[i] + y[i + 1]) * dx
    cumulative = np.cumsum(result)

    if initial is not None:
        return np.insert(cumulative, 0, initial)
    else:
        return cumulative

'''
# piano activity attempt 1 (in progress)
distance_from_origin = 0.0
az_history = []
last_step_time = None
origin_set = False
note_thresholds = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4]  # meters
note_sounds = []
note_files = ["a.wav", "b.wav", "c.wav", "d.wav", "e.wav", "f.wav", "g.wav"]
note_sounds = [pygame.mixer.Sound(f) for f in note_files]

def handle_horizontal_movement():
    global az_history, dt, origin_set, distance_from_origin, last_step_time, threshold_heel, threshold_fore

    if not origin_set:
        if int(received_heel_data) > threshold_heel or int(received_fore_data) > threshold_fore:
            print("[Piano] Origin step detected.")
            az_history = []
            origin_set = True
            last_step_time = time.time()
        return

    if int(received_heel_data) < threshold_heel and int(received_fore_data) < threshold_fore:
        az_history.append(az)

    if origin_set and (int(received_heel_data) > threshold_heel or int(received_fore_data) > threshold_fore):
        print("[Piano] Foot down again, estimating distance.")
        if len(az_history) < 2:
            return

        dt_adjusted = (time.time() - last_step_time) / len(az_history)
        velocity = cumulative_trapezoid_manual(az_history, dx=dt_adjusted, initial=0)
        displacement = cumulative_trapezoid_manual(velocity, dx=dt_adjusted, initial=0)

        distance_from_origin = abs(displacement[-1])
        print(f"[Piano] Horizontal distance: {distance_from_origin:.3f} m")

        play_note_based_on_distance(distance_from_origin)

        az_history = []
        last_step_time = time.time()

def play_note_based_on_distance(distance):
    for i, threshold in enumerate(note_thresholds):
        if distance < threshold:
            note_sounds[i].play()
            print(f"[Piano] Played note {i} for distance {distance:.2f} m")
            time.sleep(0.2)
            return
    note_sounds[-1].play()
    print(f"[Piano] Played highest note for distance {distance:.2f} m")
    time.sleep(0.2)

@app.route('/piano_start')
def piano_activity():
    print("[Piano] Starting activity...")
    _, error_response = require_selected_group()
    if error_response is not None:
        return error_response
    
    # Start the combined data thread if not already running
    start_combined_data_thread()
    
    # Reset any piano-specific variables
    global distance_from_origin, az_history, origin_set
    distance_from_origin = 0.0
    az_history = []
    origin_set = False
    
    # Start piano loop in a thread
    threading.Thread(target=piano_loop, daemon=True).start()
    
    return jsonify({"status": "Piano activity started"})

def piano_loop():
    global distance_from_origin
    while True:
        # Only run if origin is set
        if origin_set:
            handle_horizontal_movement()
        time.sleep(0.01)  

def stop_piano_activity():
    print("[Piano] Stopping activity...")
    stop_combined_data_thread()  
    global origin_set, az_history, distance_from_origin
    origin_set = False
    az_history = []
    distance_from_origin = 0.0
    print("[Piano] Activity stopped")
'''
@app.after_request
def add_no_cache_headers(response):
    if response.mimetype in ("text/html", "text/css", "application/javascript"):
        response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
    return response

#data
def start_combined_data_thread():
    global combined_data_thread, combined_data_running
    start_discovery_listener()
    if not combined_data_running:
        combined_data_running = True
        combined_data_thread = threading.Thread(target=read_combined_data)
        combined_data_thread.daemon = True
        combined_data_thread.start()

        threading.Thread(target=process_udp_data, daemon=True).start()

def stop_combined_data_thread():
    global combined_data_running
    print("Stopped")
    combined_data_running = False
    print("combined_data_running in stop thread: {}".format(combined_data_running))

def read_combined_data(): #this function to read data and other function to "process" data
    global received_heel_data, received_fore_data, received_vertical_raw, ax, ay, az, qx, qy, qz, qw, imu_orientation_mode, g, combined_data_running, R_heel, G_heel, R_fore, G_fore
    
    print("[UDP Thread] Started reading data")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1<<20)
    sock.settimeout(0.1)
    try:
        sock.bind((UDP_IP, UDP_PORT))
    except OSError as e:
        print(f"[UDP Thread] Could not bind socket: {e}")
        combined_data_running = False
        sock.close()
        return
    while combined_data_running:
        try:
            data, addr = sock.recvfrom(1024)
            record_discovered_device(addr[0])
            if len(data) < 20:
                continue
            if len(data) >= 36:
                fore_pressure, heel_pressure, ax_val, ay_val, az_val, qx_val, qy_val, qz_val, qw_val = struct.unpack('9f', data[:36])
                qx_value, qy_value, qz_value, qw_value = normalize_quaternion(qx_val, qy_val, qz_val, qw_val)
                orientation_mode = "quaternion"
            else:
                fore_pressure, heel_pressure, ax_val, ay_val, az_val = struct.unpack('5f', data[:20])
                qx_value, qy_value, qz_value, qw_value = estimate_quaternion_from_acceleration(ax_val, ay_val, az_val)
                orientation_mode = "acceleration"

            ax_meters_per_second_squared = ax_val * g
            ay_meters_per_second_squared = ay_val * g
            az_meters_per_second_squared = az_val * g

            sensor_snapshot = build_sensor_snapshot(
                fore_pressure,
                heel_pressure,
                qx_value,
                qy_value,
                qz_value,
                qw_value,
                orientation_mode,
                ax_meters_per_second_squared,
                ay_meters_per_second_squared,
                az_meters_per_second_squared,
            )
            store_sensor_snapshot(addr[0], sensor_snapshot)

            active_ip = get_active_device_ip()
            if active_ip and addr[0] != active_ip:
                continue
            # fast updates only
            R_heel = sensor_snapshot["R_heel"]
            G_heel = sensor_snapshot["G_heel"]
            R_fore = sensor_snapshot["R_fore"]
            G_fore = sensor_snapshot["G_fore"]
            received_fore_data = sensor_snapshot["fore_pressure"]
            received_heel_data = sensor_snapshot["heel_pressure"]
            qx = sensor_snapshot["qx"]
            qy = sensor_snapshot["qy"]
            qz = sensor_snapshot["qz"]
            qw = sensor_snapshot["qw"]
            imu_orientation_mode = sensor_snapshot["imu_orientation_mode"]
            ax = sensor_snapshot["ax"]
            ay = sensor_snapshot["ay"]
            az = sensor_snapshot["az"]
            received_vertical_raw = ay_val
            # push to queue (drop old if full)
            if not udp_queue.full():
                udp_queue.put_nowait((received_fore_data, received_heel_data, ax, ay, az))
        except (socket.timeout, BlockingIOError):
            continue
        except Exception as e:
            print(f"[UDP Thread] Error: {e}")
            continue
    sock.close()
    print("[UDP Thread] Stopped reading data") 

def process_udp_data(): #this function to do the slow work and the other to read as attampt to improve how fast can read data
    global received_heel_data, received_fore_data, ax, ay, az, jump_vertical_buffer, jump_collecting
    while combined_data_running:
        try:
            fore, heel, ax_val, ay_val, az_val = udp_queue.get(timeout=0.1)
            # received_fore_data = fore
            # received_heel_data = heel
            # ax, ay, az = ax_val, ay_val, az_val
            if jump_collecting:  # If jump recording is active, collect ay_val
                with jump_lock:
                    jump_vertical_buffer.append(ay_val)
            update_fore_color(fore)
            update_heel_color(heel)
            #print(f"Fore:{fore}, Heel:{heel}, ax:{ax_val:.2f}, ay:{ay_val:.2f}, az:{az_val:.2f}")
            print(f"Fore:{fore}, Heel:{heel}, ax:{ax:.2f}, ay:{ay:.2f}, az:{az:.2f}")
        except queue.Empty:
            continue

@app.route('/heel_data', methods=['GET'])
def heel_data():
    global received_heel_data
    return jsonify({'data': received_heel_data})

@app.route('/fore_data', methods=['GET'])
def fore_data():
    global received_fore_data
    return jsonify({'data': received_fore_data})

@app.route('/stop_data', methods=['GET', 'POST'])
def stop_data():
    print("[Flask] stop_data called")  
    stop_all_activities()
    return '', 204

# color
def update_heel_color(pressure):
    global R_heel, G_heel
    if pressure < 1500:
        G_heel = 255
        R_heel = int((pressure / 1000) * 255)
    elif pressure < 3000:
        R_heel = 255
        G_heel = int(255 - ((pressure - 1000) / 1000) * 255)

def update_fore_color(pressure):
    global R_fore, G_fore
    if pressure < 1000:
        G_fore = 255
        R_fore = int((pressure / 1000) * 255)
    elif pressure < 2000:
        R_fore = 255
        G_fore = int(255 - ((pressure - 100) / 1000) * 255)


def get_heel_color_channels(pressure):
    pressure_value = max(0.0, float(pressure or 0))
    if pressure_value < 1500:
        return int(max(0, min(255, (pressure_value / 1000.0) * 255))), 255
    if pressure_value < 3000:
        return 255, int(max(0, min(255, 255 - ((pressure_value - 1000.0) / 1000.0) * 255)))
    return 255, 0


def get_fore_color_channels(pressure):
    pressure_value = max(0.0, float(pressure or 0))
    if pressure_value < 1000:
        return int(max(0, min(255, (pressure_value / 1000.0) * 255))), 255
    if pressure_value < 2000:
        return 255, int(max(0, min(255, 255 - ((pressure_value - 100.0) / 1000.0) * 255)))
    return 255, 0


def build_sensor_snapshot(
    fore_pressure,
    heel_pressure,
    qx_value,
    qy_value,
    qz_value,
    qw_value,
    orientation_mode,
    ax_value=0.0,
    ay_value=0.0,
    az_value=0.0,
):
    fore_pressure_int = int(fore_pressure)
    heel_pressure_int = int(heel_pressure)
    heel_r, heel_g = get_heel_color_channels(heel_pressure_int)
    fore_r, fore_g = get_fore_color_channels(fore_pressure_int)
    return {
        "R_heel": heel_r,
        "G_heel": heel_g,
        "R_fore": fore_r,
        "G_fore": fore_g,
        "heel_pressure": heel_pressure_int,
        "fore_pressure": fore_pressure_int,
        "qx": qx_value,
        "qy": qy_value,
        "qz": qz_value,
        "qw": qw_value,
        "ax": float(ax_value),
        "ay": float(ay_value),
        "az": float(az_value),
        "imu_orientation_mode": orientation_mode,
    }


def get_default_sensor_snapshot():
    return build_sensor_snapshot(0, 0, 0.0, 0.0, 0.0, 1.0, "identity", 0.0, 0.0, 0.0)


def store_sensor_snapshot(device_ip, snapshot):
    normalized_ip = (device_ip or "").strip()
    if not normalized_ip:
        return

    snapshot_to_store = dict(snapshot)
    snapshot_to_store["device_ip"] = normalized_ip
    snapshot_to_store["updated_at"] = time.time()
    with sensor_payloads_lock:
        latest_sensor_payloads[normalized_ip] = snapshot_to_store


def get_sensor_snapshot_for_ip(device_ip):
    normalized_ip = (device_ip or "").strip()
    if not normalized_ip:
        return None

    with sensor_payloads_lock:
        snapshot = latest_sensor_payloads.get(normalized_ip)

    if snapshot is None:
        return None

    snapshot_copy = dict(snapshot)
    if time.time() - snapshot_copy.get("updated_at", 0) > DISCOVERED_DEVICE_STALE_SECONDS:
        return None

    return snapshot_copy

@app.route('/color_data', methods=['GET'])
def color_data():
    global R_heel, G_heel, R_fore, G_fore
    return jsonify({'R_heel': R_heel, 'G_heel': G_heel, 'R_fore': R_fore, 'G_fore': G_fore})


@app.route('/sensor_check_data', methods=['GET'])
def sensor_check_data():
    start_combined_data_thread()

    requested_group_id = request.args.get("group_id", "").strip()
    group = None
    if requested_group_id:
        raw_group = GROUP_OPTIONS_BY_ID.get(requested_group_id)
        if raw_group is None:
            return jsonify({
                **get_default_sensor_snapshot(),
                "group_id": requested_group_id,
                "group_label": "Unknown group",
                "device_ip": "",
                "assigned": False,
                "has_live_data": False,
                "message": "Unknown group selection.",
            }), 400
        group = build_group_option(raw_group)
    else:
        group = get_default_sensor_check_group()

    if group is None:
        return jsonify({
            **get_default_sensor_snapshot(),
            "group_id": "",
            "group_label": "No group selected",
            "device_ip": "",
            "assigned": False,
            "has_live_data": False,
            "message": "No groups are available for sensor check.",
        })

    device_ip = (group.get("ip") or "").strip()
    if not device_ip:
        return jsonify({
            **get_default_sensor_snapshot(),
            "group_id": group["id"],
            "group_label": group["label"],
            "device_ip": "",
            "assigned": False,
            "has_live_data": False,
            "message": f"{group['label']} does not have a device assigned yet.",
        })

    snapshot = get_sensor_snapshot_for_ip(device_ip)
    if snapshot is None:
        snapshot = get_default_sensor_snapshot()
        has_live_data = False
        message = f"Waiting for live data from {group['label']} on {device_ip}."
    else:
        has_live_data = True
        message = f"Checking {group['label']} on {device_ip}."

    return jsonify({
        **snapshot,
        "group_id": group["id"],
        "group_label": group["label"],
        "device_ip": device_ip,
        "assigned": True,
        "has_live_data": has_live_data,
        "message": message,
    })

# jump
def estimate_jump_height(accel_data_str):
    global dt;
    print("accel_data_str: {}".format(accel_data_str))
    if not accel_data_str:
        return 0.0
    try:
        accel_data = np.array([float(a) for a in accel_data_str])
    except ValueError:
        return 0.0
    # correct acceleration by removing leading and trailing 0s
    # check if accel_data is all 0s
    non_zero_indices = np.where(accel_data != 0)[0]
    if len(non_zero_indices) == 0:
        pass  # do nothing
    else:   
        accel_data = accel_data[non_zero_indices[0]:non_zero_indices[-1]]
    # accel_corrected = detrend(accel_data)  # Optional: Remove drift. dont reaaly need to because jump is short time
    print("accel_corrected: {}".format(accel_data))
    #accel_data = accel_corrected
    
    velocity = cumulative_trapezoid_manual(accel_data, dx=dt, initial=0)
    print("velocity: {}".format(velocity))
    dist = cumulative_trapezoid_manual(velocity, dx=dt, initial=0)
    print("displacement: {}".format(dist))

    return round(np.max(dist), 5)
'''
def get_airtime_and_height():
    global received_heel_data, received_fore_data, received_vertical_raw, vertical_raw_data_UDP, threshold_heel, threshold_fore, dt
    vertical_raw_data = []
    while True:
        if int(received_heel_data) < threshold_heel and int(received_fore_data) < threshold_fore:
            start_time = time.time()
            print("Takeoff detected")
            break
        time.sleep(0.05)
    while True:
        if int(received_heel_data) >= threshold_heel or int(received_fore_data) >= threshold_fore:
            end_time = time.time()
            stop_combined_data_thread()
            print("Landing detected")
            break
        vertical_raw_data.append(received_vertical_raw)
        time.sleep(dt) #soem time.sleep is needed but this way is not ideal
    airtime = end_time - start_time
    # print(f"Airtime: {airtime:.4f} seconds")
    #print(f"Samples collected vrdudp: {len(vertical_raw_data_UDP)}")
    #print(f"Samples collected vrd: {len(vertical_raw_data)}")
    if len(vertical_raw_data) < 10:
        print("Not enough samples for jump height. Returning 0.")
        return round(airtime, 5), 0.0
    jump_height = estimate_jump_height(vertical_raw_data) #double integration of acceleration approach (IMU)
    #jump_height = ((1/8) * 9.81) * ((airtime) ** 2) #physics formula approach. I htink this works pretty well.
    # print(f"Estimated height: {jump_height:.5f} m")
    #vertical_raw_data_UDP = []
    return round(airtime, 4), jump_height'''

def get_airtime_and_height(session_id):
    global received_heel_data, received_fore_data, jump_vertical_buffer, jump_collecting
    while True:
        if not is_activity_session_active("jump", session_id):
            jump_collecting = False
            return 0.0, 0.0, True
        if int(received_heel_data) < threshold_heel and int(received_fore_data) < threshold_fore:
            start_time = time.time()
            jump_vertical_buffer = []  # Reset buffer
            jump_collecting = True
            print("Takeoff detected")
            break
        time.sleep(0.01)
    while True:
        if not is_activity_session_active("jump", session_id):
            jump_collecting = False
            return 0.0, 0.0, True
        if int(received_heel_data) >= threshold_heel or int(received_fore_data) >= threshold_fore:
            end_time = time.time()
            jump_collecting = False
            stop_combined_data_thread()
            print("Landing detected")
            break
        time.sleep(0.01)
    airtime = end_time - start_time
    samples = []
    # Safely copy collected data
    with jump_lock:
        samples = jump_vertical_buffer[:]
    if len(samples) < 10:
        print("Not enough samples for jump height. Returning 0.")
        return round(airtime, 5), 0.0, False
    #jump_height = estimate_jump_height(samples) #calculus approach
    jump_height = ((1/8) * 9.81) * ((airtime) ** 2) #physics formula approach
    #print(f"Estimated height: {jump_height:.5f} m")
    return round(airtime, 4), jump_height, False

@app.route('/start_jump')
def start_jump():
    print("start_jump clicked")
    global last_airtime, last_jump_height, jump_metrics_ready, submitted_name2
    _, error_response = require_selected_group()
    if error_response is not None:
        return error_response
    session_id = create_activity_session("jump")
    reset_jump_state(cancel_session=False)
    start_combined_data_thread()
    airtime, height, was_cancelled = get_airtime_and_height(session_id)
    if was_cancelled:
        return jsonify({'status': 'cancelled'})
    last_airtime = airtime
    last_jump_height = height
    jump_metrics_ready = True

    # with open("SonicSoleJump.txt", "a") as f:
    #     f.write(f"{submitted_name2},{last_jump_height}\n")

    return jsonify({'status': 'jump measured'})

@app.route('/jump_metrics')
def jump_metrics():
    if not jump_metrics_ready:
        return jsonify({
            'airtime_seconds': 0.0,
            'jump_height_meters': 0.0
        })
    return jsonify({
        'airtime_seconds': last_airtime,
        'jump_height_meters': last_jump_height
    })

# balance
balance_status = {
    "status": "idle",
    "reason": "",
}


def set_balance_status(status, reason=""):
    global balance_status
    balance_status = {"status": status, "reason": reason or ""}


def balancing_pressure(session_id, device_ip):
    """Run the balance activity on the RPi and wait for the score.

    Framework contract: the webapp sends START <activity> to the RPi over
    UDP and blocks until the RPi replies with OK/ERR. All threshold
    detection and timing happen on the RPi to avoid wireless jitter.
    """
    global totalTime, recording_time

    print(f"[balance] sending START balance to RPi at {device_ip}")
    set_balance_status("measuring")
    try:
        reply = send_rpi_activity_command(device_ip, "START balance")
    except ActivityCommandError as command_error:
        print(f"[balance] command failed: {command_error.reason} ({command_error.detail})")
        if session_id == get_balance_session_id():
            recording_time = False
            set_balance_status("error", command_error.reason)
        return

    if session_id != get_balance_session_id():
        print("[balance] session changed while waiting for RPi; discarding reply")
        return

    if reply["success"] and reply.get("score") is not None:
        score_value = float(reply["score"])
        totalTime = "{:.3f}".format(score_value)
        recording_time = False
        set_balance_status("done", reply.get("reason", ""))
        print(f"[balance] score={score_value:.3f}s from RPi")
    else:
        reason = reply.get("reason") or "error"
        print(f"[balance] RPi reported failure: {reason}")
        recording_time = False
        set_balance_status("error", reason)


@app.route('/balancing', methods=['GET'])
def balancing():
    done = False
    try:
        done = not recording_time and float(totalTime) != 0
    except ValueError:
        done = False
    return jsonify({
        'data': totalTime,
        'done': done,
        'status': balance_status.get("status", "idle"),
        'reason': balance_status.get("reason", ""),
    })


@app.route('/button_click', methods=['POST'])
def button_click():
    global recording_time, totalTime
    _, error_response = require_selected_group()
    if error_response is not None:
        return error_response

    device_ip = resolve_activity_device_ip()
    if not device_ip:
        return jsonify({
            "status": "error",
            "message": "No SonicSole device is linked to the selected group yet.",
        }), 400

    session_id = create_balance_session()
    start_combined_data_thread()
    set_balance_status("countdown")
    # Visual countdown runs in the browser; audio cues are emitted by the RPi.
    time.sleep(3.12)
    if session_id != get_balance_session_id():
        set_balance_status("idle")
        return jsonify({"status": "Data transmission cancelled"})
    recording_time = True
    totalTime = "0"
    thread = threading.Thread(
        target=balancing_pressure,
        args=(session_id, device_ip),
        daemon=True,
    )
    thread.start()
    return jsonify({"status": "Data transmission started"})

# reaction time
@app.route('/start_reaction')
def start_reaction():
    global reaction_data, threshold_heel, threshold_fore, received_fore_data, received_heel_data, submitted_name2, reaction_time
    _, error_response = require_selected_group()
    if error_response is not None:
        return error_response
    device_ip = resolve_activity_device_ip()
    if not device_ip:
        return jsonify({
            "status": "error",
            "message": "No SonicSole device is linked to the selected group yet.",
        }), 400

    session_id = create_activity_session("reaction")
    reset_reaction_state(cancel_session=False)
    start_combined_data_thread()
    reaction_data = {"status": "timing", "reaction_time": 0.0}

    threading.Thread(
        target=_run_reaction_on_rpi,
        args=(session_id, device_ip),
        daemon=True,
    ).start()
    return jsonify({"status": "timing"})


def _run_reaction_on_rpi(session_id, device_ip):
    global reaction_data, reaction_time
    try:
        reply = send_rpi_activity_command(device_ip, "START reaction")
    except ActivityCommandError as command_error:
        print(f"[reaction] command failed: {command_error.reason}")
        if session_id == get_activity_session_id("reaction"):
            reaction_data = {"status": "error", "reaction_time": 0.0, "reason": command_error.reason}
        return

    if session_id != get_activity_session_id("reaction"):
        print("[reaction] session changed while waiting for RPi; discarding reply")
        return

    if reply["success"] and reply.get("score") is not None:
        score_value = round(float(reply["score"]), 4)
        reaction_time = score_value
        reaction_data = {"status": "success", "reaction_time": score_value}
        print(f"[reaction] score={score_value}s from RPi")
        return

    reason = reply.get("reason") or "error"
    if reason == "too_early":
        reaction_data = {"status": "invalid", "reaction_time": 0.0}
        print("[reaction] RPi reported: too_early")
        return

    reaction_data = {"status": "error", "reaction_time": 0.0, "reason": reason}
    print(f"[reaction] RPi reported failure: {reason}")


@app.route('/reaction_status')
def reaction_status():
    return jsonify(reaction_data)

# precision
@app.route('/start_precision_trainer')
def start_precision_trainer():
    global precision_start_time
    _, error_response = require_selected_group()
    if error_response is not None:
        return error_response
    session_id = create_activity_session("precision")
    reset_precision_trainer_state(cancel_session=False)
    precision_start_time = time.time()
    thread = Thread(target=run_precision_trainer, args=(session_id,))
    thread.start()
    return '', 200

@app.route('/precision_trainer_status')
def precision_trainer_status():
    global precision_start_time, precision_trainer_state
    if precision_start_time is None:
        return jsonify(
            status="idle",
            time=0.0,
            target_percent=0,
            target_force=0,
            current_force=0,
            current_percent=0,
            max_force=PRECISION_FORCE_MAX
        )
    elapsed = time.time() - precision_start_time
    return jsonify(
        status=precision_trainer_state['status'],
        time=elapsed,
        target_percent=precision_trainer_state.get('target_percent', 0),
        target_force=precision_trainer_state.get('target_force', 0),
        current_force=precision_trainer_state.get('current_force', 0),
        current_percent=precision_trainer_state.get('current_percent', 0),
        max_force=precision_trainer_state.get('max_force', PRECISION_FORCE_MAX)
    )

def run_precision_trainer(session_id):
    global precision_trainer_state, received_fore_data, submitted_name2, precision_error
    start_combined_data_thread()
    try:
        precision_error = 0
        target_percent = random.choice(PRECISION_TARGET_PERCENTS)
        target_force = round((target_percent / 100.0) * PRECISION_FORCE_MAX)
        precision_trainer_state['status'] = 'measuring'
        precision_trainer_state['max_force'] = PRECISION_FORCE_MAX
        precision_trainer_state['target_percent'] = target_percent
        precision_trainer_state['target_force'] = target_force
        precision_trainer_state['current_force'] = 0
        precision_trainer_state['current_percent'] = 0
        precision_trainer_state['measured_force'] = 0
        precision_trainer_state['measured_percent'] = 0
        precision_trainer_state['error_percent'] = 0

        start_time = time.time()
        while time.time() - start_time < PRECISION_CAPTURE_SECONDS:
            if not is_activity_session_active("precision", session_id):
                return
            current_force = get_precision_force_value(received_fore_data)
            current_percent = round((current_force / PRECISION_FORCE_MAX) * 100, 1)
            precision_trainer_state['current_force'] = current_force
            precision_trainer_state['current_percent'] = current_percent
            time.sleep(0.01)

        if not is_activity_session_active("precision", session_id):
            return

        measured_force = get_precision_force_value(received_fore_data)
        measured_percent = round((measured_force / PRECISION_FORCE_MAX) * 100, 1)
        precision_error = abs(measured_percent - target_percent)
        precision_trainer_state['current_force'] = measured_force
        precision_trainer_state['current_percent'] = measured_percent
        precision_trainer_state['measured_force'] = measured_force
        precision_trainer_state['measured_percent'] = measured_percent
        precision_trainer_state['error_percent'] = round(precision_error, 1)
        precision_trainer_state['status'] = 'done'
        precision_error = round(precision_error, 3)

        # with open("SonicSolePrecision.txt", "a") as f:
        #     f.write(f"{submitted_name2},{precision_error}\n")
    finally:
        stop_combined_data_thread()

@app.route('/precision_trainer_results')
def precision_trainer_results():
    return jsonify(
        target_force=precision_trainer_state.get('target_force', 0),
        measured_force=precision_trainer_state.get('measured_force', 0.0),
        measured_percent=precision_trainer_state.get('measured_percent', 0.0),
        error_percent=precision_trainer_state.get('error_percent', 0.0),
        target_percent=precision_trainer_state.get('target_percent', 0),
        max_force=precision_trainer_state.get('max_force', PRECISION_FORCE_MAX)
    )

# fore walk
@app.route('/start_forefoot', methods=['POST']) 
def start_forefoot():
    global forefoot_status, forefoot_dist, threshold_heel, dt, forefoot_elapsed_time, received_heel_data, submitted_name2
    _, error_response = require_selected_group()
    if error_response is not None:
        return error_response
    session_id = create_activity_session("forefoot")
    reset_forefoot_state(cancel_session=False)
    forefoot_status = "running"
    received_heel_data = "0"
    forefoot_dist = 0
    start_combined_data_thread()
    duration = 15.0
    start_time = time.time()
    ax_list = []

    #for vector norm (fvn)
    ay_list = []
    az_list = []

    while time.time() - start_time < duration:
        if not is_activity_session_active("forefoot", session_id):
            return jsonify({"status": "cancelled"})
        heel = int(received_heel_data)
        if heel > threshold_heel:
            forefoot_status = "invalid"
            stop_combined_data_thread()
            forefoot_elapsed_time = round(time.time() - start_time, 1)
            return jsonify({"status": "invalid", "time": forefoot_elapsed_time})
        ax_list.append(ax)

        ay_list.append(ay) #fvn
        az_list.append(az) #fvn

      #  ax_list.append(az)
        time.sleep(dt)
    stop_combined_data_thread()
    print(f"Samples collected: {len(ax_list)}")
    print(f"Samples collected: {ax_list}")

    # print(f"Samples collected: {len(ay_list)}") #fvn
    # print(f"Samples collected: {ay_list}") #fvn
    # print(f"Samples collected: {len(az_list)}") #fvn
    # print(f"Samples collected: {az_list}") #fvn

    forefoot_dist = round(estimate_distance_from_ax(ax_list,), 3)

    # forefoot_dist = round(estimate_distance_from_ax_vector_norm(ax_list, ay_list, az_list), 3) #fvn

    forefoot_status = "done"
                        
    # with open("SonicSoleWalk.txt", "a") as f:
    #     f.write(f"{submitted_name2},{forefoot_dist}\n")

    return jsonify({"status": "done", "distance_meters": forefoot_dist})

@app.route('/forefoot_status', methods=['GET'])
def get_forefoot_status():
    return jsonify({
        'status': forefoot_status,
        'distance_meters': forefoot_dist,
        'time': forefoot_elapsed_time
    })
'''
def estimate_distance_from_ax(ax_values): #later may want to add decay/kalman/low pass filter for drift
    if len(ax_values) < 2:
        return 0.0
    dt_adjusted = 15.0 / len(ax_values) #adjust dt based on num of packets actually received
    print(f"dt_adjusted: {dt_adjusted}")
    ax_array = np.array(ax_values)
    displacement = 0
    velocity = cumulative_trapezoid_manual(ax_array, dx=dt_adjusted, initial=0)
    speed = np.abs(velocity) 
    displacement = cumulative_trapezoid_manual(velocity, dx=dt_adjusted, initial=0)
   # displacement = cumulative_trapezoid_manual(speed, dx=dt_adjusted, initial=0) #distance not displacement

    return float(round(displacement[-1], 3))

'''

def estimate_distance_from_ax(ax_values):
    if len(ax_values) < 2:
        return 0.0
    dt_adjusted = 15.0 / len(ax_values) #adjust dt based on num of packets actually received
    ax_array = np.array(ax_values, dtype=np.float64)

    #  Bias Calibration (remove constant offset). This assumes user is stationary for first ~100 samples. maybe introduce calibration period.

    # bias = np.mean(ax_array[:100])  # estimate bias from first N samples
    # ax_array = ax_array - bias

    #  Low-Pass Filter (smooth out high-frequency noise)

    # alpha = 0.1  # 0 < alpha < 1, smaller = smoother
    # ax_filtered = np.zeros_like(ax_array)
    # ax_filtered[0] = ax_array[0]
    # for i in range(1, len(ax_array)):
    #     ax_filtered[i] = alpha * ax_array[i] + (1 - alpha) * ax_filtered[i - 1]
    # ax_array = ax_filtered

    #  High-Pass Filter (remove drift/very low frequency bias)

    # hp_alpha = 0.9  # closer to 1 keeps high freq, removes drift
    # ax_hp = np.zeros_like(ax_array)
    # ax_hp[0] = ax_array[0]
    # for i in range(1, len(ax_array)):
    #     ax_hp[i] = hp_alpha * (ax_hp[i-1] + ax_array[i] - ax_array[i-1])
    # ax_array = ax_hp

    #  Integration
    velocity = cumulative_trapezoid_manual(ax_array, dx=dt_adjusted, initial=0)
    #speed = np.abs(velocity) 
    #path_distance = cumulative_trapezoid_manual(speed, dx=dt_adjusted, initial=0)
    #return float(round(path_distance[-1], 3))
    displacement = cumulative_trapezoid_manual(velocity, dx=dt_adjusted, initial=0)
    return float(round(displacement[-1], 3))

def estimate_distance_from_ax_vector_norm(ax_values, ay_values, az_values): #fvn
    if len(ax_values) < 2:
        return 0.0
    dt_adjusted = 15.0 / len(ax_values) #adjust dt based on num of packets actually received
    ax_array = np.array(ax_values, dtype=np.float64)
    ay_array = np.array(ay_values, dtype=np.float64)
    az_array = np.array(az_values, dtype=np.float64)

    #  Bias Calibration (remove constant offset). This assumes user is stationary for first ~100 samples. maybe introduce calibration period.

    # bias = np.mean(ax_array[:100])  # estimate bias from first N samples
    # ax_array = ax_array - bias

    #  Low-Pass Filter (smooth out high-frequency noise)
    # this filter was made for just ax, not vector norm.
    # alpha = 0.1  # 0 < alpha < 1, smaller = smoother
    # ax_filtered = np.zeros_like(ax_array)
    # ax_filtered[0] = ax_array[0]
    # for i in range(1, len(ax_array)):
    #     ax_filtered[i] = alpha * ax_array[i] + (1 - alpha) * ax_filtered[i - 1]
    # ax_array = ax_filtered

    #  High-Pass Filter (remove drift/very low frequency bias)
    # this filter was made for just ax, not vector norm.
    # hp_alpha = 0.9  # closer to 1 keeps high freq, removes drift
    # ax_hp = np.zeros_like(ax_array)
    # ax_hp[0] = ax_array[0]
    # for i in range(1, len(ax_array)):
    #     ax_hp[i] = hp_alpha * (ax_hp[i-1] + ax_array[i] - ax_array[i-1])
    # ax_array = ax_hp

    #  Integration
    velocity_x = cumulative_trapezoid_manual(ax_array, dx=dt_adjusted, initial=0)
    velocity_y = cumulative_trapezoid_manual(ay_array, dx=dt_adjusted, initial=0)
    velocity_z = cumulative_trapezoid_manual(az_array, dx=dt_adjusted, initial=0)
    # velocity = np.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2) #vector norm for velocity
    # displacement = cumulative_trapezoid_manual(velocity, dx=dt_adjusted, initial=0)
    # return float(round(displacement[-1], 3))
    displacement_x = cumulative_trapezoid_manual(velocity_x, dx=dt_adjusted, initial=0)
    displacement_y = cumulative_trapezoid_manual(velocity_y, dx=dt_adjusted, initial=0)
    displacement_z = cumulative_trapezoid_manual(velocity_z, dx=dt_adjusted, initial=0)
    displacement_vector_norm = np.sqrt(displacement_x**2 + displacement_z**2) #vector norm for displacement
    return float(round(displacement_vector_norm[-1], 3))
    #speed_x = np.abs(velocity_x) 
    #speed_y = np.abs(velocity_y) 
    #speed_z = np.abs(velocity_z) 
    #path_distance_X = cumulative_trapezoid_manual(speed_x, dx=dt_adjusted, initial=0)
    #path_distance_Y = cumulative_trapezoid_manual(speed_y, dx=dt_adjusted, initial=0)
    #path_distance_Z = cumulative_trapezoid_manual(speed_z, dx=dt_adjusted, initial=0)
    #path_distance_vector_norm = np.sqrt(path_distance_X**2 + path_distance_Y**2 + path_distance_Z**2) #vector norm for path distance
    #return float(round(path_distance_vector_norm[-1], 3))
    

# webpage routes
@app.route('/')
def home():
    return render_template('home.html')


@app.route('/phone')
def phone_groups():
    return render_template(
        'phone_groups.html',
        phone_groups=get_group_options(),
    )


@app.route('/phone/<group_slug>')
def phone_group_home(group_slug):
    return render_phone_group_home_page(group_slug)


@app.route('/phone/<group_slug>/setup')
def phone_group_setup(group_slug):
    return render_phone_group_setup_page(group_slug)


@app.route('/phone/<group_slug>/<activity_slug>')
def phone_group_activity(group_slug, activity_slug):
    if activity_slug == "xxactivity":
        return render_phone_group_home_page(
            group_slug,
            activity_cards=PHONE_ACTIVITY_CARDS_NO_WALK,
        )

    activity = PHONE_ACTIVITY_CONFIG.get(activity_slug)
    if activity is None:
        abort(404)

    if activity_slug == "balance":
        cancel_balance_session()

    return render_phone_group_template(
        group_slug,
        activity["template"],
        activity["phone_activity"],
    )


@app.route('/<group_slug>')
def legacy_phone_group_home(group_slug):
    if get_group_from_slug(group_slug) is None:
        abort(404)
    return redirect(url_for("phone_group_home", group_slug=group_slug))


@app.route('/<group_slug>/<activity_slug>')
def legacy_phone_group_activity(group_slug, activity_slug):
    if get_group_from_slug(group_slug) is None:
        abort(404)
    if activity_slug == "setup":
        return redirect(url_for("phone_group_setup", group_slug=group_slug))
    return redirect(url_for("phone_group_activity", group_slug=group_slug, activity_slug=activity_slug))


@app.route('/hardware_status', methods=['GET'])
def hardware_status():
    start_combined_data_thread()
    live_device = get_latest_discovered_device()
    if live_device is None:
        live_device = wait_for_discovered_device(HARDWARE_STATUS_DISCOVERY_WAIT_SECONDS)

    selected_group = get_selected_group()
    selected_group_id = selected_group["id"] if selected_group is not None else None

    with ThreadPoolExecutor(max_workers=max(1, len(GROUP_SLOTS))) as executor:
        devices = list(executor.map(probe_hardware_device, GROUP_SLOTS))

    checked_at = time.time()
    for device in devices:
        device["selected"] = device["id"] == selected_group_id
        device["assignable_devices"] = get_assignable_discovered_devices(device["id"])
        device["checked_age_seconds"] = round(checked_at - device["checked_at"], 2)
        device.pop("checked_at", None)

    return jsonify({"devices": devices, "checked_at": checked_at, "live_device": live_device})

@app.route('/jump')
def jump():
    clear_selected_group()
    return render_template('jump.html')

@app.route("/precision")
def precision():
    clear_selected_group()
    return render_template("precision.html")

@app.route('/balance')
def balance():
    clear_selected_group()
    cancel_balance_session()
    return render_template('balance.html')

@app.route('/assemblyInstructions')
def assembly_instructions():
    start_combined_data_thread()
    phone_group = get_selected_group()
    return render_template(
        'assemblyInstructions.html',
        phone_group=phone_group,
        phone_group_links=build_phone_group_links(phone_group) if phone_group else None,
        sensor_check_default_group=phone_group or get_default_sensor_check_group(),
    )

@app.route('/reaction')
def reaction():
    clear_selected_group()
    return render_template('reaction.html')

@app.route('/foreWalk')
def foreWalk():
    clear_selected_group()
    return render_template('foreWalk.html')

@app.route('/piano')
def piano():
    clear_selected_group()
    return render_template('piano.html')

#plot routes


# Start the combined data thread (safe to call multiple times)
@app.route('/start_stream', methods=['GET'])
def start_stream():
    start_combined_data_thread()
    return jsonify({"status": "stream started"}), 200

# Stop the combined data thread (reuse your stop endpoint if you like)
@app.route('/stop_stream', methods=['GET'])
def stop_stream():
    stop_combined_data_thread()
    return jsonify({"status": "stream stopped"}), 200

# Return the most-recent accelerometer values
@app.route('/accel', methods=['GET'])
def accel_values():
    # ax, ay, az are global floats updated by your UDP thread
    return jsonify({
        'ax': round(ax, 4),
        'ay': round(ay, 4),
        'az': round(az, 4),
        'ts': time.time()
    })

# Render the live plotting page
@app.route('/accel_view')
def accel_view():
    start_combined_data_thread()
    return render_template('accel.html')


# scoreboards
@app.route('/scoreboard')
@app.route('/scoreboard/<board_key>')
def pc_scoreboard(board_key="jump"):
    if board_key not in PC_SCOREBOARD_KEYS:
        abort(404)

    scoreboard_rows = load_group_best_performance(board_key)
    populated_rows = [row for row in scoreboard_rows if row["has_score"]]
    leaderboard_config = get_leaderboard_config(board_key)
    scoreboard_leader = None

    if populated_rows:
        if leaderboard_config["sort_reverse"]:
            scoreboard_leader = max(populated_rows, key=lambda row: row["score"])
        else:
            scoreboard_leader = min(populated_rows, key=lambda row: row["score"])

    scoreboard_activity = {
        "slug": board_key,
        **PC_SCOREBOARD_META[board_key],
    }

    return render_template(
        'scoreboard.html',
        scoreboard_activity=scoreboard_activity,
        scoreboard_rows=scoreboard_rows,
        scoreboard_leader=scoreboard_leader,
        scoreboard_sort_reverse=leaderboard_config["sort_reverse"],
    )


@app.route('/bScoreboard')
def b_scoreboard():
    return render_template(
        'bScoreboard.html',
        data=load_balance_leaderboard(),
        embed=request.args.get("embed") == "1",
    )

@app.route('/jScoreboard')
def j_scoreboard():
    return render_template(
        'jScoreboard.html',
        data=load_metric_leaderboard("jump"),
        embed=request.args.get("embed") == "1",
    )

@app.route('/rScoreboard')
def r_scoreboard():
    return render_template(
        'rScoreboard.html',
        data=load_metric_leaderboard("reaction"),
        embed=request.args.get("embed") == "1",
    )

@app.route('/pScoreboard')
def p_scoreboard():
    return render_template(
        'pScoreboard.html',
        data=load_metric_leaderboard("precision"),
        embed=request.args.get("embed") == "1",
    )

@app.route('/wScoreboard')
def w_scoreboard():
    return render_template(
        'wScoreboard.html',
        data=load_metric_leaderboard("walk"),
        embed=request.args.get("embed") == "1",
    )


@app.route('/group_history_data', methods=['GET'])
def group_history_data():
    requested_group_id = request.args.get("group_id", "").strip()
    if not requested_group_id:
        requested_group_id = session.get("selected_group_id", "")

    group = GROUP_OPTIONS_BY_ID.get(requested_group_id)
    if group is None:
        return jsonify({"status": "error", "message": "Unknown group selection."}), 400

    selected_group = build_group_option(group)
    try:
        history = load_group_history(selected_group["label"])
        sample_payload = None
        if not history_has_attempts(history):
            sample_payload = load_sample_group_history(group)

        if sample_payload is not None:
            sample_group = sample_payload.get("group") if isinstance(sample_payload.get("group"), dict) else {}
            return jsonify(
                {
                    "status": "ok",
                    "group": {**selected_group, **sample_group},
                    "history": sample_payload["history"],
                    "source": "sample",
                }
            )

        return jsonify(
            {
                "status": "ok",
                "group": selected_group,
                "history": history,
                "source": "live",
            }
        )
    except Exception:
        app.logger.exception("Failed to load group history for %s", requested_group_id)
        sample_payload = load_sample_group_history(group)
        if sample_payload is not None:
            sample_group = sample_payload.get("group") if isinstance(sample_payload.get("group"), dict) else {}
            return jsonify(
                {
                    "status": "ok",
                    "group": {**selected_group, **sample_group},
                    "history": sample_payload["history"],
                    "source": "sample",
                }
            )

        return (
            jsonify(
                {
                    "status": "error",
                    "message": "Unable to load score history right now.",
                    "group": selected_group,
                    "history": [],
                }
            ),
            500,
        )


@app.route('/leaderboard/<board_key>/update', methods=['POST'])
def update_leaderboard_entry(board_key):
    entry_key = request.form.get("entry_key", "").strip()
    if not entry_key:
        return redirect_to_leaderboard(board_key)

    if board_key == "balance":
        name = get_group_name_from_request("group_name")
        best_time = parse_score(request.form.get("best_time"))

        if not name or best_time is None:
            return redirect_to_leaderboard(board_key)

        updated_entries = []
        for entry in load_balance_leaderboard():
            if entry["entry_key"] == entry_key:
                updated_entries.append(
                    {
                        "entry_key": name,
                        "name": name,
                        "best_time": best_time,
                    }
                )
            else:
                updated_entries.append(entry)

        write_leaderboard_rows("balance", serialize_balance_leaderboard(updated_entries))
        return redirect_to_leaderboard(board_key)

    config = get_leaderboard_config(board_key)
    name = get_group_name_from_request("group_name")
    value = parse_score(request.form.get("value"))
    if not name or value is None:
        return redirect_to_leaderboard(board_key)

    updated_entries = []
    for entry in load_metric_leaderboard(board_key):
        if entry["entry_key"] == entry_key:
            updated_entries.append(
                {
                    "entry_key": name,
                    "name": name,
                    config["value_field"]: value,
                }
            )
        else:
            updated_entries.append(entry)

    write_leaderboard_rows(board_key, serialize_metric_leaderboard(board_key, updated_entries))
    return redirect_to_leaderboard(board_key)


@app.route('/leaderboard/<board_key>/delete', methods=['POST'])
def delete_leaderboard_entry(board_key):
    entry_key = request.form.get("entry_key", "").strip()
    if not entry_key:
        return redirect_to_leaderboard(board_key)

    if board_key == "balance":
        remaining_entries = [
            entry for entry in load_balance_leaderboard()
            if entry["entry_key"] != entry_key
        ]
        write_leaderboard_rows("balance", serialize_balance_leaderboard(remaining_entries))
        return redirect_to_leaderboard(board_key)

    remaining_entries = [
        entry for entry in load_metric_leaderboard(board_key)
        if entry["entry_key"] != entry_key
    ]
    write_leaderboard_rows(board_key, serialize_metric_leaderboard(board_key, remaining_entries))
    return redirect_to_leaderboard(board_key)


@app.route('/select_group', methods=['POST'])
def select_group():
    payload = request.get_json(silent=True) or request.form
    group_id = payload.get("group_id", "").strip()
    group = GROUP_OPTIONS_BY_ID.get(group_id)
    if group is None:
        return jsonify({"status": "error", "message": "Unknown group selection."}), 400

    selected_group = set_selected_group(group_id, activate_device=True)
    return jsonify({"status": "ok", "group": selected_group})

@app.route('/assign_group_device', methods=['POST'])
def assign_group_device():
    payload = request.get_json(silent=True) or request.form
    group_id = payload.get("group_id", "").strip()
    device_ip = payload.get("device_ip", "").strip()
    group = GROUP_OPTIONS_BY_ID.get(group_id)
    if group is None:
        return jsonify({"status": "error", "message": "Unknown group selection."}), 400

    if not device_ip:
        return jsonify({"status": "error", "message": "Choose a SonicSole device first."}), 400

    start_combined_data_thread()
    assignable_devices = get_assignable_discovered_devices(group_id)
    if not any(device["ip"] == device_ip for device in assignable_devices):
        conflicting_group_id = get_group_id_for_assigned_ip(device_ip, exclude_group_id=group_id)
        if conflicting_group_id is not None:
            conflicting_group = GROUP_OPTIONS_BY_ID.get(conflicting_group_id)
            return jsonify(
                {
                    "status": "error",
                    "message": f"{device_ip} is already assigned to {conflicting_group['label']}.",
                }
            ), 409
        return jsonify(
            {
                "status": "error",
                "message": f"{device_ip} is not currently available for assignment.",
            }
        ), 400

    assigned_group, conflicting_group = assign_group_device_ip(group_id, device_ip)
    if conflicting_group is not None:
        return jsonify(
            {
                "status": "error",
                "message": f"{device_ip} is already assigned to {conflicting_group['label']}.",
            }
        ), 409

    selected_group = get_selected_group()
    if selected_group is not None and selected_group["id"] == group_id:
        set_active_device_ip(assigned_group["ip"])

    return jsonify(
        {
            "status": "ok",
            "group": assigned_group,
            "device_ip": device_ip,
            "message": f"Assigned {device_ip} to {assigned_group['label']}.",
        }
    )


@app.route('/disconnect_group_device', methods=['POST'])
def disconnect_group_device():
    payload = request.get_json(silent=True) or request.form
    group_id = payload.get("group_id", "").strip()
    group = GROUP_OPTIONS_BY_ID.get(group_id)
    if group is None:
        return jsonify({"status": "error", "message": "Unknown group selection."}), 400

    disconnected_group, _ = assign_group_device_ip(group_id, "")
    selected_group = get_selected_group()
    if selected_group is not None and selected_group["id"] == group_id:
        set_active_device_ip(None)

    return jsonify(
        {
            "status": "ok",
            "group": disconnected_group,
            "message": f"Disconnected {group['label']}.",
        }
    )


@app.route('/submitB', methods=['POST'])
def submitB():
    global submitted_name, totalTime
    submitted_name = get_group_name_from_request("group_name")

    append_leaderboard_row("balance", submitted_name, totalTime)
    return jsonify({"status": "Group name submitted successfully"})

# submit2 is unused
# @app.route('/submit2', methods=['POST'])
# def submit2():
#     global submitted_name2, first_name2, last_name2
#     first_name2 = request.form['first_name2']
#     last_name2 = request.form['last_name2']
#     submitted_name2 = first_name2 + " " + last_name2
#     return jsonify({"status": "Name submitted successfully"})
# recording_time = True

@app.route('/submitR', methods=['POST'])
def submitR():
    global submitted_name2, reaction_time, forefoot_dist
    submitted_name2 = get_group_name_from_request("group_name")

    append_leaderboard_row("reaction", submitted_name2, reaction_time)

    return jsonify({"status": "Group name submitted successfully"})

@app.route('/submitW', methods=['POST'])
def submitW():
    global submitted_name2, reaction_time
    submitted_name2 = get_group_name_from_request("group_name")

    append_leaderboard_row("walk", submitted_name2, forefoot_dist)

    return jsonify({"status": "Group name submitted successfully"})


@app.route('/submitP', methods=['POST'])
def submitP():
    global submitted_name2, precision_error
    submitted_name2 = get_group_name_from_request("group_name")

    append_leaderboard_row("precision", submitted_name2, precision_error)

    return jsonify({"status": "Group name submitted successfully"})

@app.route('/submitJ', methods=['POST'])
def submitJ():
    global submitted_name2, last_jump_height
    submitted_name2 = get_group_name_from_request("group_name")
    
    append_leaderboard_row("jump", submitted_name2, last_jump_height)

    return jsonify({"status": "Group name submitted successfully"})


# misc (not sure what these are for)
def send_udp_data(): 
    target_ip = get_active_device_ip()
    if not target_ip:
        return False
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    n = 1
    sock.sendto(n.to_bytes(1, byteorder='big'), (target_ip, UDP_PORT))
    sock.close()
    return True

@app.route('/button', methods=['POST'])
def button():
    send_udp_data()
    return redirect(url_for('jump'))

# main
if __name__ == '__main__':
    start_discovery_listener()
    flask_port = int(os.environ.get("SONICSOLE_PORT", os.environ.get("PORT", "5001")))
    app.run(host='0.0.0.0', port=flask_port, debug=False)
