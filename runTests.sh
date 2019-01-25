#!/bin/bash

function join { local IFS="$1"; shift; echo "$*"; }

export var_file=$(mktemp)

function write_to {
  local file=$(mktemp)
  echo "${1}" >> "${var_file}"
  echo "${file}" >> "${var_file}"
  echo ${file}
};

function parse_write {
  local i
  local lines

  readarray lines < "${var_file}"
  : > "${var_file}"

  for (( i = 0; i < ${#lines[@]}; i += 2 )); do
    local var
    local file

    var=$(tr -d '\n' <<< "${lines[${i}]}")
    file=$(tr -d '\n' <<< "${lines[$(( ${i} + 1 ))]}")

    printf -v "${var}" "%s" "$(cat ${file})"
    rm -fr "${file}"
  done
};

for size in 25 50 100 250 500 750 1000; do
  for deg in 3 5 10; do
    for rep in 1 2 3 4 5 6 7 8 9 10; do
      echo -n "size = ${size}, average degree = ${deg}, repetition = ${rep} . . ."
      src/cplex -input instances/akcan-like/a-$size-$deg-$rep.dat -nadir instances/akcan-like/nadir/a-$size-$deg-$rep.csv -objective 1 > $(write_to csv_end);
      echo " Done!"
      parse_write
      echo -en "\r\033[K"
      echo "${csv_start}${csv_end}" | tee results/a-$size-$deg-$rep.dat
    done
  done
done
