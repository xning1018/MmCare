import urllib.request, json, time
for i in range(5):
    req = urllib.request.Request('http://182.254.135.92:8080/api/data/latest?api_key=mmcare-demo-key-2026', headers={'X-API-Key': 'mmcare-demo-key-2026'})
    d = json.loads(urllib.request.urlopen(req, timeout=5).read().decode('utf-8'))['data']
    print(f"{d.get('timestamp')}: h_m={d.get('h_m')}, v60={d.get('v60_ms')}, vd={d.get('v_drop_ms')}, dist={d.get('distance')}, speed={d.get('motion_speed')}")
    time.sleep(1.0)
