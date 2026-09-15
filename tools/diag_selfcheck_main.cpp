// Supporting unit-level evidence for the Diagnostics tab's word grouping
// (multi-word generalisation, dispatched alongside SWE1-GUI-007's GNSS-trust
// amendment): exercises diagSelfCheck() -- the same buildDiagWordGroups()
// logic XcpPanel::rebuildDiagTable() uses -- without a display, a GL context
// or a QApplication. diagmath.cpp only touches QString/QVector/A2lMeas value
// types plus A2lModel::parseMeasurements (file I/O against a QTemporaryFile,
// no network), so a plain main() is enough; this deliberately does NOT
// instantiate XcpPanel/QTableWidget (those need a real widget context), only
// the free functions/structs the Diagnostics tab is built from.
//
// Built and run by build.bat (local) and .github/workflows/build.yml (CI) --
// see both for the exact invocation. Non-zero exit / any [FAIL] line means
// the diagnostics word-grouping logic regressed.
#include "diagmath.h"

#include <QString>
#include <cstdio>

int main()
{
    QString report;
    const bool ok = diagSelfCheck(&report);

    std::fputs(qUtf8Printable(report), stdout);
    std::fputs(ok ? "[diag_selfcheck] PASS\n" : "[diag_selfcheck] FAIL\n", stdout);

    return ok ? 0 : 1;
}
