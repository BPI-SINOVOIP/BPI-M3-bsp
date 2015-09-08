
die() {
        echo "$*" >&2
        exit 1
}

[ -s "./chosen_board.mk" ] || die "please run ./configure first."

set -e

. ./chosen_board.mk

PACK_ROOT="sunxi-pack"
#PLATFORM="linux"
PLATFORM="dragonboard"

echo "MACH=$MACH, PLATFORM=$PLATFORM, BOARD=$BOARD"

cd $PACK_ROOT
./pack -c $MACH -p $PLATFORM -b $BOARD 
cd -
