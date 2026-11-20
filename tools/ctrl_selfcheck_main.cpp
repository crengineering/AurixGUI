// Headless self-check for the flight-controller panel's pure logic (read-back
// windows, parser, status decode, write gates, yaw unwrap) -- see ctrlmath.h.
// No display, no QApplication. Built and run by build.bat and CI; non-zero exit
// / any [FAIL] line means the logic regressed.
#include "ctrlmath.h"

#include <cstdio>

int main()
{
    QString report;
    const bool ok = ctrlSelfCheck(&report);
    std::fputs(qUtf8Printable(report), stdout);
    std::fputs(ok ? "[ctrl_selfcheck] PASS\n" : "[ctrl_selfcheck] FAIL\n", stdout);
    return ok ? 0 : 1;
}
