#!/usr/bin/env bash

PRINT_USAGE=0
EXIT_CODE=0

if [ -z $1 ] ; then
  PRINT_USAGE=1
  echo "No arguments provided. Printing usage."
  EXIT_CODE=1
elif [ $1 = '-h' ] || [ $1 = '--help' ]; then
  PRINT_USAGE=1
elif [ ! -x $1 ] ; then
  PRINT_USAGE=1
  EXIT_CODE=2
  echo "Binary provided was not executable. Printing usage."
fi

if [ $PRINT_USAGE -eq 0 ] ; then

  BIN=$1
  DIR=${BIN}_deps
  SCRIPT=${BIN}.sh
  ZIP=${BIN}.zip

  DEPS=$(ldd $BIN | cut -d' ' -f3 | egrep "(/opt|/home|pcl)")

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

else 

  echo "Program summary for $0 <binary>:"
  echo ""
  echo "  Zips <binary> dependencies and creates an executable .sh file for"
  echo "  the binary specified by <binary>. The results can be copied to other"
  echo "  hosts to execute. Three files/dirs will be created:"
  echo ""
  echo "    <binary>_deps (directory, can be safely deleted)"
  echo "    <binary>.sh (shell script to execute on other hosts - NEEDED)"
  echo "    <binary>.zip (zip of all dependencies - NEEDED)"
fi

exit $EXIT_CODE