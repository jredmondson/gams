#!/usr/bin/env bash

BIN=$1
DIR=${BIN}_deps
SCRIPT=${BIN}.sh
ZIP=${BIN}.zip

DEPS=$(ldd $BIN | cut -d' ' -f3 | egrep "(/opt|/home)")

mkdir -p $DIR

for DEP in $DEPS; do
  DEP_FILE=$(basename $DEP)
  cp $DEP $DIR/$DEP_FILE
done

cat > $SCRIPT <<EOF
#!/usr/bin/env bash

WD="\$( cd "\$( dirname "\${BASH_SOURCE[0]}" )" && pwd )"

export LD_LIBRARY_PATH="\$WD/${DIR}:$LD_LIBRARY_PATH"
\$WD/$BIN "\$@"
EOF

chmod +x $SCRIPT

zip $ZIP $BIN $DIR/* $SCRIPT
