#!/bin/bash
for dir in $(ls patches); do
  if [ $dir == "apply.sh" ]; then continue; fi
  echo "pathching $dir"
  git -C $dir apply ../patches/$dir
done
