#ifdef PORTHOROLO_8M
   #include "ESP32_LOLIN_S2_Mini.h"
   #include "fc_pointsofsale_8m.h"
#endif

#ifdef PORTHOROLO_2M
   #include "fts2som.h"
   #include "fcporthorolo2m.h"
#endif

#ifdef PORTHOROLO_4M_WPCC
   #include "fts2som.h"
   #include "fcporthorolo4m.h"
#endif

#ifdef DEV_DINGY_4M
   #include "fts2mini.h"
   #include "fcdevdingy4m.h"
#endif

#ifdef DEV_DINGY_6M
   #include "fts2mini.h"
   #include "fcdevdingy6m.h"
#endif

#ifdef DEV_DINGY_16M
   #include "fts2mini.h"
   #include "fcdevdingy16m.h"
#endif

#ifdef DEV_DINGY_6M_PROTO
   #include "fts2miniproto.h"
   #include "fcdevdingy6mproto.h"
#endif


#ifdef DD_FLAGSHIP_16M
   #include "fts2som.h"
   #include "fcddflagship16m.h"
#endif
