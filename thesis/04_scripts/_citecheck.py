import re

tex = open('thesis/smarmplus_main.tex', encoding='utf-8').read()
keys = []
for m in re.finditer(r'\\cite[a-z]*\*?\{([^}]*)\}', tex):
    for k in m.group(1).split(','):
        k = k.strip()
        if k and k not in keys:
            keys.append(k)
bib = open('thesis/reference.bib', encoding='utf-8').read()
entries = re.split(r'\n(?=@)', bib)
bk = {}
for e in entries:
    m = re.match(r'\s*@(\w+)\{([^,]+),', e)
    if m:
        bk[m.group(2).strip()] = e
print('cited', len(keys), 'bib', len(bk))
print('MISSING in bib:', [k for k in keys if k not in bk])
print('UNCITED bib:', [k for k in bk if k not in keys])
for i, k in enumerate(keys, 1):
    e = bk.get(k, '')
    t = re.search(r'title\s*=\s*[{"](.+?)[}"],?\s*\n', e)
    has = 'doi' in e.lower()
    print(f'[{i}] {k} | doi={has} | {t.group(1)[:70] if t else ""}')

