import gzip
import base64

with open('dashboard.html', 'rb') as f:
    content = f.read()

compressed = gzip.compress(content)
b64 = base64.b64encode(compressed).decode('ascii')

cmd = f'python3 -c "import gzip, base64; d = gzip.decompress(base64.b64decode(\'{b64}\')); open(\'/home/ubuntu/index.html\', \'wb\').write(d); open(\'/home/ubuntu/dashboard.html\', \'wb\').write(d)"'

with open('deploy_cmd.txt', 'w', encoding='utf-8') as f:
    f.write(cmd)

print("SUCCESS! Updated deploy_cmd.txt, size:", len(cmd))
