#!/usr/bin/env bash
# Test for `host/doctor.sh --json`, the machine-readable form the fleet status
# agent reads. Runs the real doctor with --no-uart (never touches the serial
# port, never starts a container), so it works on a Jetson, a laptop or CI:
# the checks themselves may PASS or FAIL, the test only asserts the output
# contract. Needs python3 (or python) to parse the JSON.
#
#   bash host/tests/test_doctor_json.sh
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCTOR="${HERE}/../doctor.sh"

TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

pass=0
fail=0
ok()   { printf '  ok   %s\n' "$*"; pass=$((pass + 1)); }
bad()  { printf '  FAIL %s\n' "$*"; fail=$((fail + 1)); }
check() { if [[ "$2" == "$3" ]]; then ok "$1 ($2)"; else bad "$1: expected '$3', got '$2'"; fi; }

PY=""
for c in python3 python; do   # Windows' python3 can be a Store stub that only prints
  if command -v "${c}" >/dev/null 2>&1 && "${c}" -c 'import json' >/dev/null 2>&1; then PY="${c}"; break; fi
done
if [[ -z "${PY}" ]]; then
  echo "[test] python3 not found -- cannot parse JSON"; exit 1
fi
# Native path for python on Git Bash (C:/...), unchanged elsewhere.
native() { if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else printf '%s' "$1"; fi; }

# A team workspace path with a quote, a backslash and a tab: it ends up in the
# team-ws detail, so it exercises the escaping.
WEIRD=$'nonexistent-team-ws/we"ird\\path\twith tab'
rc_json=0
TEAM_WS="${WEIRD}" SKYBETA_DRONE_ENV="${TMP}/none.env" \
  bash "${DOCTOR}" --json --no-uart >"${TMP}/out.json" 2>"${TMP}/err.txt" || rc_json=$?
rc_text=0
TEAM_WS="${WEIRD}" SKYBETA_DRONE_ENV="${TMP}/none.env" \
  bash "${DOCTOR}" --no-uart >"${TMP}/out.txt" 2>/dev/null || rc_text=$?

summary="$(WEIRD="${WEIRD}" "${PY}" - "$(native "${TMP}/out.json")" <<'PY'
import json, os, sys
try:
    d = json.load(open(sys.argv[1], encoding="utf-8"))
except Exception as e:
    print("parse=%s" % type(e).__name__); raise SystemExit(0)
print("parse=ok")
print("schema=%s" % d.get("schema"))
print("keys=%s" % ",".join(k for k in ("date", "host", "image", "version", "args", "checks",
                                       "pass", "warn", "fail", "ok") if k in d))
c = d.get("checks", [])
print("n_checks=%d" % len(c))
print("results_valid=%s" % all(x.get("result") in ("PASS", "WARN", "FAIL") for x in c))
print("counts_match=%s" % ((d["pass"], d["warn"], d["fail"]) ==
      tuple(sum(1 for x in c if x["result"] == r) for r in ("PASS", "WARN", "FAIL"))))
print("ok_matches_fail=%s" % (d["ok"] == (d["fail"] == 0)))
print("args=%s" % " ".join(d["args"]))
print("pwf=%d/%d/%d" % (d["pass"], d["warn"], d["fail"]))
print("ok=%s" % d["ok"])
ws = [x for x in c if x["name"] == "team-ws"]
print("weird_roundtrip=%s" % bool(ws and ws[0]["detail"].startswith(os.environ["WEIRD"] + " ")))
print("no_ansi=%s" % ("\x1b" not in open(sys.argv[1], encoding="utf-8").read()))
PY
)"
get() { sed -n "s/^$1=//p" <<<"${summary}"; }

check "stdout is one JSON object"            "$(get parse)" "ok"
check "schema"                               "$(get schema)" "1"
check "top-level keys"                       "$(get keys)" "date,host,image,version,args,checks,pass,warn,fail,ok"
check "13 checks (no --boot-camera)"         "$(get n_checks)" "13"
check "every result is PASS/WARN/FAIL"       "$(get results_valid)" "True"
check "pass/warn/fail = the checks' counts"  "$(get counts_match)" "True"
check "ok == (fail == 0)"                    "$(get ok_matches_fail)" "True"
check "args recorded"                        "$(get args)" "--json --no-uart"
check "quote/backslash/tab survive escaping" "$(get weird_roundtrip)" "True"
check "no ANSI colour codes"                 "$(get no_ansi)" "True"
check "exit status agrees with ok"           "$([[ "$(get ok)" == True ]] && echo 0 || echo 1)" "${rc_json}"
check "same exit status as the table"        "${rc_json}" "${rc_text}"

# The text table must be unchanged in content: same counts as the JSON.
trailer="$(sed -n 's/^doctor: \([0-9]*\) PASS  \([0-9]*\) WARN  \([0-9]*\) FAIL$/\1\/\2\/\3/p' "${TMP}/out.txt")"
check "table trailer matches JSON counts"    "${trailer}" "$(get pwf)"
check "table still has its header"           "$(head -c 16 "${TMP}/out.txt")" "skybeta doctor -"

if grep -q 'uart-traffic' "${TMP}/out.json" && grep -q 'skipped (--no-uart)' "${TMP}/out.json"; then
  ok "--no-uart honoured in JSON mode"
else
  bad "--no-uart: uart-traffic not reported as skipped"
fi

echo
echo "[test] ${pass} passed, ${fail} failed"
[[ "${fail}" -eq 0 ]] || exit 1
exit 0
