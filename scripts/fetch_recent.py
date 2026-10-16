import urllib.request
import json

try:
    req = urllib.request.urlopen('http://182.254.135.92:8080/api/data/history?limit=20', timeout=5)
    res = json.loads(req.read().decode('utf-8'))
    items = res.get('data', [])
    for d in items[-20:]:
        print(f"{d.get('timestamp')} | pers:{d.get('person')} | pose:{d.get('pose'):<10} | h:{d.get('h_m')} | dist:{d.get('distance')} | fall:{d.get('fall')} | v_drop:{d.get('v_drop_ms')}")
except Exception as e:
    print("Error:", e)
