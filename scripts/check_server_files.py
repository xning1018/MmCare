import urllib.request

for fname in ['mmcare_server/schemas.py', 'mmcare_server/services/readings.py', 'mmcare_server/api/legacy.py']:
    content = urllib.request.urlopen(f'http://182.254.135.92:8000/{fname}', timeout=5).read().decode('utf-8')
    has_hm = 'h_m' in content
    print(f"{fname}: has 'h_m'? -> {has_hm}")
