#!/usr/bin/env python3
"""
Guard the web UI against the API contract.

The checks in AGENTS.md 5.2 (HTML parse, `node --check`, duplicate identifiers)
verify that the UI is *syntactically* valid. They cannot catch a UI that is
syntactically perfect but offers a value the firmware rejects -- which is
exactly how the 2.048 MSPS option survived all the way to a 400 in the field
(LEARNING.md L36).

This script cross-checks the values the UI can send against the constraints
enforced in main/http_radar.c, so that class of drift fails before a flash.
"""
import os
import re
import sys
from html.parser import HTMLParser

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
UI = os.path.join(ROOT, "main", "radar_data", "radar.html")
API = os.path.join(ROOT, "main", "http_radar.c")

failures = []
checks = 0


def check(cond, msg):
    global checks
    checks += 1
    if not cond:
        failures.append(msg)


def main():
    ui = open(UI, encoding="utf-8").read()
    api = open(API, encoding="utf-8").read()

    # ---- 1. The single most important invariant ----------------------------
    # The API accepts ONLY 2400000 and 400s on anything else. Because the
    # settings form POSTs every field together, a UI that can emit any other
    # value breaks the entire save, not just the rate field.
    rejects_non_2400 = re.search(
        r"sample_rate_sps\s*!=\s*-1\s*&&\s*sample_rate_sps\s*!=\s*2400000", api
    )
    check(rejects_non_2400 is not None,
          "API no longer appears to reject non-2400000 sample rates; update this test")

    if rejects_non_2400:
        # No <option> may offer a rate the API rejects.
        opts = re.findall(r'<option value="(\d+)"[^>]*>([^<]*)</option>', ui)
        for value, label in opts:
            if re.match(r"^(2048|2400|2000|1024|512|256)", value) or "MSPS" in label or "SPS" in label:
                check(value == "2400000",
                      f"UI <option value=\"{value}\"> ({label.strip()}) offers a sample "
                      f"rate the API rejects; a settings save would 400")

        # No JS literal may propose a rejected rate either (alert "fix" payloads).
        for lit in re.findall(r"sample_rate_sps\s*:\s*(\d+)", ui):
            check(lit == "2400000",
                  f"UI posts sample_rate_sps={lit}, which the API rejects with 400")

        check("FIXED_SAMPLE_RATE" in ui,
              "UI should pin the sample rate via FIXED_SAMPLE_RATE so a stale DOM "
              "cannot 400 the whole settings form")

    # ---- 2. Gain range: UI and API must agree ------------------------------
    m = re.search(r"gain\s*!=\s*-1\s*&&\s*\(gain\s*<\s*(\d+)\s*\|\|\s*gain\s*>\s*(\d+)\)", api)
    check(m is not None, "could not find the gain range check in the API")
    if m:
        lo, hi = int(m.group(1)), int(m.group(2))
        for lit in re.findall(r"gain\s*:\s*(\d+)", ui):
            check(lo <= int(lit) <= hi,
                  f"UI posts gain={lit}, outside the API range {lo}-{hi}")
        for lit in re.findall(r"Math\.(?:min|max)\((\d+),", ui):
            check(lo <= int(lit) <= hi,
                  f"UI clamps against {lit}, outside the API range {lo}-{hi}")
        attrs = re.search(r'id="setGain"[^>]*min="(\d+)"[^>]*max="(\d+)"', ui)
        if attrs:
            check((int(attrs.group(1)), int(attrs.group(2))) == (lo, hi),
                  f"UI gain input is {attrs.group(1)}-{attrs.group(2)} but the API "
                  f"enforces {lo}-{hi}")

    # ---- 3. Syntax checks (the pre-existing AGENTS.md 5.2 gate) -----------
    class P(HTMLParser):
        def __init__(self):
            super().__init__()
            self.void = {"br", "img", "input", "meta", "link", "hr", "source",
                         "area", "base", "col", "embed", "param", "track", "wbr"}
            self.stack, self.err = [], []

        def handle_starttag(self, t, a):
            if t not in self.void:
                self.stack.append(t)

        def handle_endtag(self, t):
            if t in self.void:
                return
            if self.stack and self.stack[-1] == t:
                self.stack.pop()
            else:
                self.err.append(f"mismatched </{t}>")

    p = P()
    p.feed(ui)
    check(not p.err, f"HTML parse errors: {p.err}")
    check(not p.stack, f"unclosed HTML tags: {p.stack}")

    ids = re.findall(r'\bid="([^"]+)"', ui)
    dupes = {i for i in ids if ids.count(i) > 1}
    check(not dupes, f"duplicate element ids: {sorted(dupes)}")

    print(f"ran {checks} checks")
    if failures:
        print("\nFAILED:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("all UI/API contract checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
