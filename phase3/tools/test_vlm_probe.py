"""An unavailable Pi must not bypass the intended 'not checked' branch."""
import re,subprocess,sys
from pathlib import Path
p=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[2]/'rover'
s=p.read_text();section=s[s.index('\nvlm)'):s.index('\n# ── up:')]
assignment=re.search(r'  hz=\$\(timeout 30 ssh.*?awk.*?\)(?: \|\| hz="")?',section,re.S).group(0)
for value,code,expected in [('',255,'not_checked'),('/camera/color/image_raw/compressed 4.2',0,'4.2')]:
 script='set -euo pipefail\nLAPTOP_USER=x; PI5_HOST=x; DOMAIN=42\n'
 script+=f"ssh() {{ printf '%s\\n' '{value}'; return {code}; }}\ntimeout() {{ shift; \"$@\"; }}\n"
 script+=assignment+'\nif [ -z "$hz" ]; then echo not_checked; else echo "$hz"; fi\necho continued\n'
 r=subprocess.run(['bash','-c',script],text=True,capture_output=True)
 assert r.returncode==0,(r.returncode,r.stderr)
 assert r.stdout.splitlines()==[expected,'continued'],r.stdout
print('VLM unreachable/reachable probe regressions PASS')
