// Supporting unit-level evidence for SWE1-GUI-006/-007/-008 (vv: PIL -- the
// bench acceptance is the qualifying level; this covers only the pure logic
// underneath it): exercises positionSelfCheck() -- the same window-scaling/
// sign-flip/anchoring-truth-table logic the Position panel's paintEvent()s
// use -- without a display, a GL context or a QApplication. positionmath.cpp
// only touches QString/
// QPointF value types, so a plain main() is enough; this deliberately does
// NOT instantiate PositionView/PositionTrailWidget/AltitudeBarWidget (those
// need a real widget/A2L/XCP context), only the free functions they share.
//
// Built and run by build.bat (local) and .github/workflows/build.yml (CI) --
// see both for the exact invocation. Non-zero exit / any [FAIL] line means
// the position math regressed.
#include "positionmath.h"

#include <QString>
#include <cstdio>

int main()
{
    QString report;
    const bool ok = positionSelfCheck(&report);

    std::fputs(qUtf8Printable(report), stdout);
    std::fputs(ok ? "[position_selfcheck] PASS\n" : "[position_selfcheck] FAIL\n", stdout);

    return ok ? 0 : 1;
}
