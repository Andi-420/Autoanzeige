#!/usr/bin/env python3
"""
Bildet die Prototyp-Erzeugung der Arduino IDE nach (vereinfacht):
Fuer jede Funktion auf oberster Ebene wird ein Prototyp erzeugt und alle
Prototypen werden VOR der ersten Funktionsdefinition eingefuegt.

So faellt im Simulator derselbe Fehler auf wie in der IDE, z.B. wenn ein
eigener Typ (struct) erst nach der ersten Funktion definiert wird, aber in
einer Funktionssignatur vorkommt ("'X' does not name a type").

Aufruf: arduino_proto.py <eingabe.ino> <ausgabe.cpp>
"""
import re
import sys

KEYWORDS = ("if", "for", "while", "switch", "return", "else", "do", "struct",
            "typedef", "enum", "class", "extern", "namespace", "using", "case")


def strip_comment(line):
    i = line.find("//")
    return line if i < 0 else line[:i]


def find_functions(lines):
    """Liefert [(zeilenindex, prototyp)] fuer Funktionsdefinitionen auf oberster Ebene."""
    out = []
    i = 0
    depth = 0          # Klammertiefe { } ausserhalb von Funktionskoepfen
    while i < len(lines):
        line = lines[i]
        code = strip_comment(line)
        starts = (depth == 0 and re.match(r"[A-Za-z_]", line)
                  and not re.match(r"(%s)\b" % "|".join(KEYWORDS), line)
                  and "(" in code)
        if starts:
            # Signatur ggf. ueber mehrere Zeilen einsammeln
            text, j = code, i
            while not re.search(r"[{;]", text) and j + 1 < len(lines):
                j += 1
                text += " " + strip_comment(lines[j]).strip()
            open_p = text.find("(")
            lvl, k = 0, open_p
            for k in range(open_p, len(text)):
                if text[k] == "(":
                    lvl += 1
                elif text[k] == ")":
                    lvl -= 1
                    if lvl == 0:
                        break
            rest = text[k + 1:].lstrip()
            if lvl == 0 and rest.startswith("{") and "=" not in text[:open_p]:
                sig = " ".join(text[:k + 1].split())
                # Default-Argumente gehoeren nur an eine Stelle
                sig = re.sub(r"\s*=\s*[^,()]+(?=[,)])", "", sig)
                out.append((i, sig + ";"))
            # Klammertiefe fuer die eingesammelten Zeilen nachfuehren
            for t in lines[i:j + 1]:
                c = strip_comment(t)
                depth += c.count("{") - c.count("}")
            i = j + 1
            continue
        depth += code.count("{") - code.count("}")
        i += 1
    return out


def main():
    src, dst = sys.argv[1], sys.argv[2]
    lines = open(src, encoding="utf-8").read().split("\n")
    funcs = find_functions(lines)
    if funcs:
        first = funcs[0][0]
        protos = ["// --- automatisch erzeugte Prototypen (wie Arduino IDE) ---"]
        protos += [p for _, p in funcs]
        protos += ["#line %d" % (first + 1)]
        lines = lines[:first] + protos + lines[first:]
    open(dst, "w", encoding="utf-8").write("\n".join(lines))


if __name__ == "__main__":
    main()
