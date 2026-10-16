import gzip
import base64

with open('dashboard.html', 'rb') as f:
    raw_data = f.read()

compressed = gzip.compress(raw_data)
b64_str = base64.b64encode(compressed).decode('ascii')

deploy_cmd = (
    f"python3 -c \"import gzip, base64; "
    f"d = gzip.decompress(base64.b64decode('{b64_str}')); "
    f"open('/home/ubuntu/index.html', 'wb').write(d); "
    f"open('/home/ubuntu/dashboard.html', 'wb').write(d)\""
)

with open('deploy_cmd.txt', 'w', encoding='utf-8') as f:
    f.write(deploy_cmd)

print(f"deploy_cmd.txt successfully updated! Total length: {len(deploy_cmd)} chars.")
