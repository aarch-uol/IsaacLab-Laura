#!/bin/bash
# scr/mods/bash_utils.sh


log()    { 
    echo -e "$1" | tee -a "logs/bash_tracker.md"
}


python() {
  local start end elapsed mins secs
  start=$(date +%s)

  log "## Script: \"$*\" \n[Started]:  @ $(date -d @$start '+%m-%d %H:%M:%S')"
  /workspace/isaaclab/_isaac_sim/python.sh "$@"

  end=$(date +%s)
  elapsed=$((end - start))
  mins=$((elapsed / 60))
  secs=$((elapsed % 60))

  log "## Script: \"$*\" \n[Finished]: @ $(date -d @$end '+%m-%d %H:%M:%S')"
  log "**Elapsed Time**: ${mins}m ${secs}s"

  chown -R 1002:1002 /workspace/isaaclab/
}

loop() {
    args=("$@")
    local cmd=("${args[@]}")

    for algo in "${algos[@]}"; do 
        log "\n# Playing $algo for $task dir: $dir\n"
        for x in $(seq "$start" "$step" "$stop"); do
            "${cmd[@]}"

        done
    done
}

export -f log python