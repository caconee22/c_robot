import re, subprocess, tempfile, os
s = open(r'C:\Users\CACONEE\AppData\Local\Temp\claude\7c908481c573955d\27e7ba6c-e533-416a-9420-8b0208cecf50\scratchpad\irc-schedule.html', encoding='utf-8').read()
for i, m in enumerate(re.findall(r'<script[^>]*>(.*?)</script>', s, flags=re.S)):
    p = os.path.join(tempfile.gettempdir(), f'chk{i}.js')
    open(p, 'w', encoding='utf-8').write(m)
    r = subprocess.run(['node', '--check', p], capture_output=True, text=True, encoding='utf-8')
    print(i, len(m), r.returncode, r.stderr[:300])
