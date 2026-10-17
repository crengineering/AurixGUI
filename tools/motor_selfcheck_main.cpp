// Headless self-check for the Motors tab's pure logic (slider <-> DShot
// mapping: never 1..47 on the wire, STOP position, bar scaling, state names).
// No display, no QApplication -- see motormath.h. Built and run by build.bat
// and CI; non-zero exit / any [FAIL] line means the mapping regressed.
#include "motormath.h"

#include <cstdio>

int main()
{
    QString report;
    const bool ok = motorSelfCheck(&report);
    std::fputs(qUtf8Printable(report), stdout);
    std::fputs(ok ? "[motor_selfcheck] PASS\n" : "[motor_selfcheck] FAIL\n", stdout);
    return ok ? 0 : 1;
}
