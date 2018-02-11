#define RBUF_SIZE 1024

typedef struct  ringBufS
{
  char *buf;
  int head;
  int tail;
  int count;
} ringBufS;

void  ringBufS_init  (ringBufS *_this);
void  ringBufS_free  (ringBufS *_this);
int   ringBufS_empty (ringBufS *_this);
int   ringBufS_full  (ringBufS *_this);
char  ringBufS_get   (ringBufS *_this);
void  ringBufS_put   (ringBufS *_this, const char c);
void  ringBufS_flush (ringBufS *_this, const int clearBuffer);
