#ifndef TB45_CELL_INPUT_BRIDGE_H
#define TB45_CELL_INPUT_BRIDGE_H

#ifdef __cplusplus
#include <string>

extern std::string CELLAPN;
extern std::string CELLUN;
extern std::string CELLPW;
extern std::string CELLPIN;
extern "C" {
#endif

const char *tb45_main_get_cellapn(void);
const char *tb45_main_get_cellun(void);
const char *tb45_main_get_cellpw(void);
const char *tb45_main_get_cellpin(void);

#ifdef __cplusplus
}
#endif

#endif /* TB45_CELL_INPUT_BRIDGE_H */

