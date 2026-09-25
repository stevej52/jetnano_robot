# Copyright 2026 stevej52
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""What's going on: headlines, the weather and the markets, in a few sentences.

    ros2 run jetnano_bringup listen --news all        # or world, us, local, weather, markets

No keys needed: Google News RSS for headlines, Open-Meteo for the weather,
Yahoo Finance's chart endpoint for the indexes, and ip-api to learn where
"local" is when no location is set. Everything is short: three headlines a
section, one sentence of weather, one of markets. ``chunks()`` cuts a
briefing into pieces of about thirty seconds of speech.
"""

import html
import json
import re
import urllib.parse
import urllib.request

UA = {'User-Agent': 'Mozilla/5.0 (Rosie the robot; jetnano)'}
GNEWS = 'https://news.google.com/rss'
TAIL = '?hl=en-US&gl=US&ceid=US:en'
WORDS_PER_CHUNK = 70          # about thirty seconds at her speaking pace


def _get(url: str, timeout: float = 8.0) -> str:
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read().decode('utf-8', 'replace')


# ----------------------------------------------------------------- where --

def where(location: str = ''):
    """(city, region, lat, lon): the named place, else where the robot's
    internet connection appears to be."""
    if location:
        j = json.loads(_get('https://geocoding-api.open-meteo.com/v1/search?count=1&language=en&name='
                            + urllib.parse.quote(location)))
        r = j['results'][0]
        return r['name'], r.get('admin1', ''), r['latitude'], r['longitude']
    j = json.loads(_get('http://ip-api.com/json/?fields=status,city,regionName,lat,lon'))
    if j.get('status') != 'success':
        raise RuntimeError('no location')
    return j['city'], j['regionName'], j['lat'], j['lon']


# ------------------------------------------------------------- headlines --

def _clean(title: str) -> str:
    t = html.unescape(title)
    t = re.sub(r'\s+[-|]\s+[^-|]{2,40}$', '', t)      # " - The Publisher"
    t = t.replace('’', "'").replace('‘', "'").replace('“', '"').replace('”', '"')
    t = re.sub(r'[\[\]{}<>*_#]', '', t)
    t = re.sub(r'\s+', ' ', t).strip()
    if t and t[-1] not in '.!?':
        t += '.'
    return t


def headlines(section: str, n: int = 3, city: str = '') -> list:
    """section: world | us | local | business | top."""
    if section == 'top':
        url = GNEWS + TAIL
    elif section == 'local':
        url = f'{GNEWS}/headlines/section/geo/{urllib.parse.quote(city)}{TAIL}'
    else:
        topic = {'world': 'WORLD', 'us': 'NATION', 'business': 'BUSINESS'}[section]
        url = f'{GNEWS}/headlines/section/topic/{topic}{TAIL}'
    # titles by pattern, not an XML parser: nothing from the network gets to
    # expand entities here
    out = []
    for raw in re.findall(r'<item>.*?<title>(.*?)</title>', _get(url), re.S):
        raw = re.sub(r'^\s*<!\[CDATA\[(.*?)\]\]>\s*$', r'\1', raw, flags=re.S)
        t = _clean(raw)
        if len(t) > 8 and t not in out:
            out.append(t)
        if len(out) == n:
            break
    return out


# --------------------------------------------------------------- weather --

WMO = [(0, 'clear'), (1, 'mostly clear'), (2, 'partly cloudy'), (3, 'overcast'), (45, 'foggy'), (48, 'foggy'),
       (51, 'drizzling'), (53, 'drizzling'), (55, 'drizzling'), (56, 'freezing drizzle'), (57, 'freezing drizzle'),
       (61, 'raining lightly'), (63, 'raining'), (65, 'raining hard'), (66, 'freezing rain'), (67, 'freezing rain'),
       (71, 'snowing lightly'), (73, 'snowing'), (75, 'snowing hard'), (77, 'sleeting'),
       (80, 'showery'), (81, 'showery'), (82, 'pouring'), (85, 'snow showers'), (86, 'snow showers'),
       (95, 'thundery'), (96, 'thundery with hail'), (99, 'thundery with hail')]


def weather(city: str, lat: float, lon: float) -> str:
    url = ('https://api.open-meteo.com/v1/forecast?latitude={}&longitude={}'
           '&current=temperature_2m,weather_code,wind_speed_10m'
           '&daily=temperature_2m_max,temperature_2m_min,precipitation_probability_max'
           '&temperature_unit=fahrenheit&wind_speed_unit=mph&timezone=auto&forecast_days=1').format(lat, lon)
    j = json.loads(_get(url))
    cur, day = j['current'], j['daily']
    sky = dict(WMO).get(int(cur['weather_code']), 'unsettled')
    s = f"It's {round(cur['temperature_2m'])} degrees and {sky} in {city}. "
    s += f"Today's high is {round(day['temperature_2m_max'][0])} and the low {round(day['temperature_2m_min'][0])}"
    rain = day.get('precipitation_probability_max', [None])[0]
    if rain is not None:
        s += f', with a {int(rain)} percent chance of rain'
    s += '.'
    wind = cur.get('wind_speed_10m')
    if wind is not None and wind >= 10:
        s += f' Wind is {round(wind)} miles an hour.'
    return s


# --------------------------------------------------------------- markets --

INDEXES = (('^GSPC', 'The S and P 500'), ('^DJI', 'the Dow'), ('^IXIC', 'the Nasdaq'))


def markets() -> str:
    parts = []
    for sym, name in INDEXES:
        try:
            j = json.loads(_get(f'https://query1.finance.yahoo.com/v8/finance/chart/{urllib.parse.quote(sym)}'
                                '?interval=1d&range=5d'))
            meta = j['chart']['result'][0]['meta']
            price = float(meta['regularMarketPrice'])
            prev = float(meta.get('chartPreviousClose') or meta.get('previousClose') or 0)
            if prev:
                pct = (price - prev) / prev * 100.0
                parts.append(f'{name} is at {price:.0f}, {"up" if pct >= 0 else "down"} {abs(pct):.1f} percent')
            else:
                parts.append(f'{name} is at {price:.0f}')
        except Exception:      # noqa: BLE001 - any one index may be unavailable
            continue
    if not parts:
        return "I couldn't get the market numbers."
    return ', '.join(parts) + '.'


# -------------------------------------------------------------- briefing --

KINDS = ('world', 'us', 'local', 'weather', 'markets')


def briefing(kind: str = 'all', location: str = '') -> list:
    """Sentences to say. kind: all | world | us | local | weather | markets."""
    kinds = list(KINDS) if kind == 'all' else [kind]
    per = 2 if kind == 'all' else 3
    out = []
    place = None
    if 'local' in kinds or 'weather' in kinds:
        try:
            place = where(location)
        except Exception:      # noqa: BLE001
            place = None
    for k in kinds:
        try:
            if k == 'world':
                out += ['World news.'] + headlines('world', per)
            elif k == 'us':
                out += ['U S news.'] + headlines('us', per)
            elif k == 'local':
                if place:
                    out += [f'News from {place[0]}.'] + headlines('local', per, place[0])
                else:
                    out.append("I don't know where local is.")
            elif k == 'weather':
                out.append(weather(place[0], place[2], place[3]) if place else "I couldn't find out where we are for the weather.")
            elif k == 'markets':
                out.append(markets())
        except Exception as exc:      # noqa: BLE001 - one dead source must not kill the briefing
            out.append(f"I couldn't get the {k} {'news' if k in ('world', 'us', 'local') else ''}".strip() + '.')
            _ = exc
    return out


def chunks(sentences: list, words: int = WORDS_PER_CHUNK) -> list:
    """Group sentences into pieces of about `words` words; a section title
    stays with its first headline."""
    out, cur, n = [], [], 0
    for s in sentences:
        w = len(s.split())
        if cur and n + w > words and not cur[-1].endswith('news.') and not cur[-1].startswith('News from'):
            out.append(cur)
            cur, n = [], 0
        cur.append(s)
        n += w
    if cur:
        out.append(cur)
    return out


def main(argv) -> int:
    kind = argv[0] if argv else 'all'
    location = argv[1] if len(argv) > 1 else ''
    import time
    t0 = time.monotonic()
    sents = briefing(kind, location)
    print(f'{kind}: {len(sents)} sentences in {time.monotonic() - t0:.1f} s, {len(chunks(sents))} chunk(s)')
    for i, c in enumerate(chunks(sents), 1):
        print(f'--- chunk {i} ({sum(len(s.split()) for s in c)} words)')
        for s in c:
            print('  ' + s)
    return 0
