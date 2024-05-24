#include "lowpassfilter.h"

void lp_filter(lp_t *lp, uint32_t raw) {
  lp->smooth = lp->smooth - (lp->beta * (lp->smooth - raw));
}
