import curses
import os
import sys
import json
import agrparse
from datetime import datetime
from pathlib import Path

# Setup
BUILD_SCRIPT = "./build.sh"
BUILD_BINARY = "./build/lifelong" 
MR_DIR = "./MR24"
OUTPUT_DIR = "./output"
ROOT = "."

def rel(path:str) -> str:
    return str(Path(ROOT)/path)

def find_problems_files() -> list:
    d = Path(rel(MR_DIR))
    if not d.exists():
        return []
    return sorted(d.rglob("*.json"))

def exists_output_dir():
    os.makedirs(rel(OUTPUT_DIR), exist_ok=True) 
    
def timestamped_output(problem_file: Path)