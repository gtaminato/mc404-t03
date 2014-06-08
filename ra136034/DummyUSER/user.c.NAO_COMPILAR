/*
 user.c
 -
 Este programa cria vários processos idênticos com fork(). Neste exemplo,
 cada processo roda o mesmo código C presente neste arquivo, mas cada processo
 irá rodar trechos diferentes desse código, criando a diferenciação entre eles.

 Normalmente, em sistemas UNIX, uma segunda chamada de sistema execve() é 
 chamada para executar outro programa no novo processo criado com fork(). Mas
 não utilizaremos execve() aqui para simplificar o projeto do sistema
 operacional.

 Autor: Rafael Auler, 21/Nov/2011
 */

#include <unistd.h>

# ifndef __pid_t_defined
typedef unsigned pid_t;
#  define __pid_t_defined
#endif

/* ========================================================================= */
/* Aqui está o código da computação realizada pelo primeiro caso de 
   processo criado com fork(). Neste caso, realizaremos a
   computação de Fibonacci.*/
int fib(int n) 
{
  if (n < 2)
    return n;
  else
    return fib(n-1) + fib(n-2);
}

/* Converte um inteiro para string. Retorna o número de bytes escritos no
   buffer de destino. */
unsigned int2string(char *dst, int num) {
  char buf[256];
  int p = 0, i;
  unsigned num_bytes = 0;
  if (num < 0) {
    *(dst++) = '-';
    num = ~num + 1;
    ++num_bytes;
  }	
  do {
    int rem = num % 10;
    num = num / 10;
    char c = '0' + rem;
    buf[p++] = c;
  } while (num != 0);
  for (i = 0; i < p; ++i) {
    *(dst++) = buf[p-1-i];
    ++num_bytes;
  }
  return num_bytes;
}

/* Copia uma string para outra string de destino. Retorna o número de bytes
   escritos no buffer de destino. */
unsigned str2str(char *dst, char *src) {
  unsigned num_bytes = 0;
  while (*src != '\0') {
    *(dst++) = *(src++);
    ++num_bytes;
  }
  return num_bytes;
}

void adeus() {
  char buf[200];
  char *pbuf = buf;
  pbuf += str2str(pbuf, "PID ");
  pbuf += int2string(pbuf, (int) getpid());
  pbuf += str2str(pbuf, ": Processo encerrando.\n");
  write(1, buf, (int) (pbuf - buf));    
  _exit(0);
}

void codigo_processo_1(int param_fib) {
  /* Calcula F(100) em que F é a sequência de Fibonacci */
  char buf[200];
  char *pbuf = buf;
  int res = fib(param_fib);
  pbuf += str2str(pbuf, "PID ");
  pbuf += int2string(pbuf, (int) getpid());
  pbuf += str2str(pbuf, ": Fibonacci(");
  pbuf += int2string(pbuf, param_fib);
  pbuf += str2str(pbuf, ") = ");
  pbuf += int2string(pbuf, res);
  pbuf += str2str(pbuf, "\n");
  write(1, buf, (int) (pbuf - buf));
  adeus();
}

/* ========================================================================= */
/* Aqui está o código da computação realizada pelo segundo caso de processo
   criado com fork(). Neste caso, realizaremos um laço que periodicamente
   imprime mensagens na tela.*/
void codigo_processo_2(int param_for) {
  int i, j;
  for (i = 0; i < 10; ++i) {
    for (j = 0; j < param_for; ++j) {
    }
    char buf[200];
    char *pbuf = buf;
    pbuf += str2str(pbuf, "PID ");
    pbuf += int2string(pbuf, (int) getpid());
    pbuf += str2str(pbuf, ": Estou vivo!\n");
    write(1, buf, (int) (pbuf - buf));    
  }
  adeus();
}

/* Ponto de entrada */
int main() {
  pid_t num_proc = 0;

  char buf[200];
  char *pbuf = buf;
  pbuf += str2str(pbuf, "PID ");
  pbuf += int2string(pbuf, (int) getpid());
  pbuf += str2str(pbuf, ": Criando processos filhos.\n");
  write(1, buf, (int) (pbuf - buf));    

  /* Cria primeiro processo filho */
  num_proc = fork();
  if (num_proc == 0)
    codigo_processo_1(30);

  /* Cria outros processos filhos */
  num_proc = fork();
  if (num_proc == 0)
    codigo_processo_2(1000000);

  num_proc = fork();
  if (num_proc == 0)
    codigo_processo_1(31);

  num_proc = fork();
  if (num_proc == 0)
    codigo_processo_2(2000000);

  num_proc = fork();
  if (num_proc == 0)
    codigo_processo_1(32);

  num_proc = fork();
  if (num_proc == 0)
    codigo_processo_2(1500000);

  num_proc = fork();
  if (num_proc == 0)
    codigo_processo_1(28);


  adeus();
}
