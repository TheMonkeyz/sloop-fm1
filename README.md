# Snoop37 mirror: the built installer site

https://themonkeyz.github.io/sloop-fm1/

Built from the branch `snoop37-mirror` at dd5d747:

    BOARD=smk37 SNOOP_MIRROR=1 (tools/build.py)  ->  build/sloop-smk37.fwsc
    python web/make_site.py build/sloop-smk37.fwsc 2.8.5-mirror OUT

`index.html` is written by hand and lives only in this branch. GPL-3.0, as the source.
