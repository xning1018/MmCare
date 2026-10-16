import gzip
import base64

with open("dashboard.html", "rb") as f:
    raw = f.read()

b64 = base64.b64encode(gzip.compress(raw)).decode("ascii")

cmd = f'''python3 -c "import gzip, base64; d = gzip.decompress(base64.b64decode('{b64}')); open('/home/ubuntu/index.html', 'wb').write(d); open('/home/ubuntu/dashboard.html', 'wb').write(d)"'''

with open("deploy_cmd.txt", "w", encoding="utf-8") as out:
    out.write(cmd)

print("Deploy command generated, length:", len(cmd))
