#!/usr/bin/env bash
ROOT="${1:-.}"
BINARY="$ROOT/build/lifelong"
PROBLEMS_DIR="$ROOT/MR24"
OUTPUT_DIR="$ROOT/output"

prompt() {
    read -rp "$1 [$2]: " val
    echo "${val:-$2}"
}

do_compile() {
    echo "=== Compile & Build ==="
    (cd "$ROOT" && bash build.sh)
    read -rp "Press Enter to continue..."
}

do_run() {
    echo "=== Select Problem Domain ==="
    select domain in "$PROBLEMS_DIR"/*/; do
        [[ -n "$domain" ]] && break
    done

    echo "=== Select Problem Instance ==="
    select problem in "$domain"*.json; do
        [[ -n "$problem" ]] && break
    done

    echo "=== Hyperparameters (Enter to keep default) ==="

    # derive output path
    domain_name=$(basename "$domain" .domain)
    problem_name=$(basename "$problem" .json)
    number=$(echo "$problem_name" | grep -oP '(?<=-)\d+' | sed 's/^0*//')

    if [[ -n "$number" ]]; then
        default_out="$OUTPUT_DIR/$domain_name/$number"
    else
        default_out="$OUTPUT_DIR/$domain_name"
    fi

    SIM=$(prompt     "Simulation time"      "5")
    PRE_TL=$(prompt  "Preprocess TL"        "600000")
    PLAN_TL=$(prompt "Plan time limit"      "60000")
    CLUSTERS=$(prompt "Num of clusters (Large Map)"        "400")
    RADIUS=$(prompt   "Radius DBC"              "3")
    PENALTY=$(prompt  "cPenalty"            "2")
#    MODE=$(prompt     "Mode (wppl/pibt)"                "wppl")
    WINDOW=$(prompt   "Window"              "10")
    HORIZON=$(prompt  "Horizon"             "1")
    THREADS=$(prompt  "Thread Workers (Cores)"             "3")
    OUT_NAME=$(prompt "Output name"          "$problem_name")

    out_file="$default_out/$OUT_NAME.json"
    mkdir -p "$default_out"

    "$BINARY" \
        --inputFile "$problem" \
        -o "$out_file" \
        --simulationTime "$SIM" \
        --planTimeLimit "$PLAN_TL" \
        --preprocessTimeLimit "$PRE_TL" \
        --numberOfCluster "$CLUSTERS" \
        --radius "$RADIUS" \
        --cPenalty "$PENALTY" \
        --mode "$MODE" \
        --window "$WINDOW" \
        --horizon "$HORIZON" \
        --threads "$THREADS"

    echo
    echo "Output saved to: $out_file"
    read -rp "Press Enter to continue..."
}

PS3="Choice: "
while true; do
    echo
    echo "=== MAPF Start-Kit ==="
    select opt in Compile Run Quit; do
        case $opt in
            Compile) do_compile; break ;;
            Run)     do_run;     break ;;
            Quit)    exit 0 ;;
        esac
    done
done
