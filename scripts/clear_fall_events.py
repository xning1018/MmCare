import urllib.request
import json

url = 'http://182.254.135.92:8080/api/v1/fall/events?device_id=MM001&only_unhandled=true&limit=500'
req = urllib.request.urlopen(url)
raw = json.loads(req.read().decode('utf-8'))
events = raw.get('data', [])
print(f"Total unhandled events: {len(events)}")

count = 0
for e in events:
    eid = e['id']
    post_url = f'http://182.254.135.92:8080/api/v1/fall/events/{eid}/handle'
    h_req = urllib.request.Request(post_url, data=b'', method='POST')
    try:
        urllib.request.urlopen(h_req)
        count += 1
    except Exception as err:
        print(f"Failed to handle {eid}: {err}")

print(f"Successfully handled {count} fall events.")

# 验证 latest 读数
latest_url = 'http://182.254.135.92:8080/api/data/latest'
latest_data = json.loads(urllib.request.urlopen(latest_url).read().decode('utf-8'))
print("Latest API Data:", json.dumps(latest_data.get('data', {}), indent=2))
