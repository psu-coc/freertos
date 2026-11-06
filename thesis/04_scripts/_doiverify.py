import difflib
import json
import re
import time
import unicodedata
import urllib.error
import urllib.parse
import urllib.request

UA = {'User-Agent': 'smarmplus-revision/1.0 (mailto:research@example.org)'}
bib = open('thesis/reference.bib', encoding='utf-8').read()


def field(e, name):
    m = re.search(r'\b' + name + r'\s*=\s*\{((?:[^{}]|\{(?:[^{}]|\{[^{}]*\})*\})*)\}', e, re.S)
    return re.sub(r'\s+', ' ', m.group(1)).strip() if m else ''


def norm(s):
    s = s.replace('\\"', '').replace("\\'", '').replace('\\`', '')
    s = unicodedata.normalize('NFKD', s)
    s = ''.join(c for c in s if not unicodedata.combining(c))
    s = re.sub(r'<[^>]+>', '', s)
    s = re.sub(r'[{}\\]', '', s)
    s = s.replace('&amp;', '&')
    return re.sub(r'[^a-z0-9]', '', s.lower())


def bib_families(a):
    fams = []
    for p in re.split(r'\s+and\s+', a):
        p = p.strip()
        if not p:
            continue
        fam = p.split(',')[0] if ',' in p else p.split()[-1]
        fams.append(norm(fam))
    return fams


def get(url):
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode('utf-8'))


rows = []
for e in re.split(r'\n(?=@)', bib):
    m = re.match(r'\s*@(\w+)\{([^,]+),', e)
    if not m:
        continue
    key = m.group(2).strip()
    doi = field(e, 'doi')
    if not doi:
        continue
    note = field(e, 'note')
    issues = []
    nd = re.search(r'doi: \\url\{([^}]*)\}', note)
    if not nd or nd.group(1) != doi:
        issues.append(f'NOTE-DOI MISMATCH ({nd.group(1) if nd else "missing"})')
    time.sleep(1.0)
    try:
        w = get('https://api.crossref.org/works/' + urllib.parse.quote(doi))['message']
        src = 'crossref'
        title = w.get('title', [''])[0]
        fams = [norm(a.get('family', '')) for a in w.get('author', [])]
        year = (w.get('issued', {}).get('date-parts') or [[None]])[0][0]
        pages, vol, iss = w.get('page'), w.get('volume'), w.get('issue')
    except urllib.error.HTTPError:
        try:
            d = get('https://api.datacite.org/dois/' + urllib.parse.quote(doi))['data']['attributes']
            src = 'datacite'
            title = d['titles'][0]['title']
            fams = [norm(c.get('familyName', c.get('name', ''))) for c in d.get('creators', [])]
            year = d.get('publicationYear')
            pages = vol = iss = None
        except Exception as ex:
            rows.append(f'{key}\t{doi}\tRESOLVE-FAIL {ex}')
            continue
    bt = field(e, 'title')
    sim = difflib.SequenceMatcher(None, norm(bt), norm(title)).ratio()
    if sim < 0.9 and not (norm(title) and norm(title) in norm(bt)):
        issues.append(f'TITLE sim={sim:.2f} | reg="{title[:80]}"')
    bf = bib_families(field(e, 'author'))
    if fams and bf != fams:
        issues.append(f'AUTHORS bib={bf} reg={fams}')
    by = field(e, 'year')
    if year and str(year) != by:
        issues.append(f'YEAR bib={by} reg={year}')
    bp = field(e, 'pages').replace('--', '-')
    if pages and bp and bp != pages:
        issues.append(f'PAGES bib={bp} reg={pages}')
    if vol and field(e, 'volume') and field(e, 'volume') != str(vol):
        issues.append(f'VOLUME bib={field(e, "volume")} reg={vol}')
    if iss and field(e, 'number') and field(e, 'number') != str(iss):
        issues.append(f'NUMBER bib={field(e, "number")} reg={iss}')
    rows.append(f'{key}\t{doi}\t{src}\t' + ('OK' if not issues else ' ; '.join(issues)))

open('thesis/_doi_verify_report.txt', 'w', encoding='utf-8').write('\n'.join(rows))
print(len(rows), 'checked;', sum('OK' not in r.split('\t')[-1] for r in rows), 'with issues')
