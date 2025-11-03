Academic Year: 2025/2026

# Report on Particle Filter assignment development

### Step 1: implementazione di codice funzionante
Sono riuscito ad implementare tutte le funzioni richieste, tuttavia il risultato lascia molto a desiderare in quanto quasi subito tutte le particelle collassano in un singolo punto. Ho provato a smanettare con i parametri:

``` bash
double sigma_init [3] = {0.80, 0.80, 0.20};  //[x,y,theta] initialization noise. -> lets try 0.8 meters on x,y and 0.2 rads on theta
double sigma_pos [3]  = {0.03, 0.03, 0.01}; //[x,y,theta] movement noise. Try values between [0.5 and 0.01]
double sigma_landmark [2] = {0.8, 0.8};     //[x,y] sensor measurement noise. Try values between [0.5 and 0.1]
```
sia inizializzando con initial guess che random, il problema persiste.

### Step 2: implementazione di resampling alternativo

Dato che il problema è che le particelle collassano rapidamente in un unica particella (50 uguali) ho provato a implementare un metodo alternativo di resampling in cui a una percentuale minoritaria di particelle resemplate viene aggiunto del rumore.

![result](/3_particle_filter/first_prototype.png)

