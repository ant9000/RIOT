#!/bin/bash

BASE=$(dirname $0)


test_config() {
     if [ ! `which edbg` ]; then
        echo "###################################################"
        echo "Error: Command 'edbg' is missing"
        echo "Please install with:"
        echo "   git clone https://github.com/ant9000/edbg && \ "
        echo "   cd edbg && make && sudo cp edbg /usr/local/bin"
        echo "###################################################"
        exit -1
     fi
}

test_hexfile() {
    if [ ! -f "${HEXFILE}" ]; then
        echo "Error: Unable to locate HEXFILE"
        echo "       (${HEXFILE})"
        exit 1
    fi
}

do_flash() {
    test_config
    test_hexfile
    if [ -n "${PRE_FLASH_CHECK_SCRIPT}" ]; then
        sh -c "${PRE_FLASH_CHECK_SCRIPT} '${HEXFILE}'"
        RETVAL=$?
        if [ $RETVAL -ne 0 ]; then
            echo "pre-flash checks failed, status=$RETVAL"
            exit $RETVAL
        fi
    fi
    # flash device
    BINFILE=${HEXFILE%.hex}.bin
    python $BASE/ihex.py $HEXFILE > $BINFILE
    edbg -b -t atmel_cm0p -e -p -v -f $BINFILE
    echo 'Done flashing'
}

ACTION="$1"
shift # pop $1 from $@

case "${ACTION}" in
  flash)
    echo "### Flashing Target ###"
    do_flash "$@"
    ;;
  *)
    echo "Usage: $0 {flash}"
    ;;
esac

