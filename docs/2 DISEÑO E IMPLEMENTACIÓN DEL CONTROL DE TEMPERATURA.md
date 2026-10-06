**UNIVERSIDAD NACIONAL DE INGENIERÍA**

**FACULTAD DE INGENIERÍA ELECTRICA Y ELECTRÓNICA**

![Universidad Nacional de Ingeniería - José Manuel Castillo Cara](data:image/png;base64...)

**TESIS:**

**DISEÑO E IMPLEMENTACIÓN DEL CONTROL DE TEMPERATURA PARA TANQUES DE MEZCLADO CONTINÚO BASADO EN UN PREDICTOR SMITH**

**PARA OPTAR EL GRADO ACADÉMICO DE MAESTRO EN CIENCIAS CON MENCIÓN EN AUTOMÁTICA E INSTRUMENTACIÓN**

**ELABORADO POR:**

**JORGE ENRIQUE ORTIZ PORRAS**

**ASESOR:**

**M. Sc. RICARDO RODRIGUEZ BUSTINZA**

**LIMA – PERÚ**

**2021**

# **DEDICATORIA**

**ÍNDICE DE CONTENIDOS**

[DEDICATORIA 2](#_Toc85212155)

[ÍNDICE DE FIGURAS 5](#_Toc85212156)

[RESUMEN 8](#_Toc85212157)

[INTRODUCCIÓN 1](#_Toc85212158)

[**CAPÍTULO I 2**](#_Toc85212159)

[**ASPECTOS INTRODUCTORIOS 2**](#_Toc85212160)

[1.1. Antecedentes bibliográficos 2](#_Toc85212161)

[1.2. Descripción del problema 4](#_Toc85212162)

[1.2.1. ¿Qué es un Retardo? 5](#_Toc85212163)

[1.2.2. Sistemas de Control para procesos con Retardo 5](#_Toc85212164)

[1.2.3. Predictor de Smith 6](#_Toc85212165)

[1.2.4. Limitaciones del Predictor de Smith 7](#_Toc85212166)

[1.3. Formulación del problema 7](#_Toc85212167)

[1.4. Justificación e importancia de la investigación 8](#_Toc85212168)

[1.5. Objetivos 8](#_Toc85212169)

[1.5.1. Objetivo General 8](#_Toc85212170)

[1.5.2. Objetivos Específicos 8](#_Toc85212171)

[1.6. Hipótesis 8](#_Toc85212172)

[1.6.1. Hipótesis principal 8](#_Toc85212173)

[1.7. Variables e indicadores 8](#_Toc85212174)

[1.7.1. Variable independiente 8](#_Toc85212175)

[1.7.2. Variable dependiente 8](#_Toc85212176)

[1.7.3. Indicadores 9](#_Toc85212177)

[1.8. Unidad de análisis 9](#_Toc85212178)

[1.9. Tipo y nivel de Investigación 9](#_Toc85212179)

[1.10. Periodo de análisis: 9](#_Toc85212180)

[1.11. Fuentes de información e instrumentos utilizados: 10](#_Toc85212181)

[1.11.1. Caudalímetro 10](#_Toc85212182)

[1.11.2. Predictor Smith 10](#_Toc85212183)

[1.11.3. Matlab 11](#_Toc85212184)

[1.12. Técnicas de recolección y procesamiento de datos 11](#_Toc85212185)

[1.12.1. Garantizar una buena gestión de datos 13](#_Toc85212186)

[**CAPÍTULO II** 14](#_Toc85212187)

[**MARCO TEÓRICO Y CONCEPTUAL** 14](#_Toc85212188)

[2.1. Características generales de los tanques industriales 14](#_Toc85212189)

[2.1.1. Tipos de tanques de mezclado 17](#_Toc85212190)

[2.1.1.1. Agitadores de paletas 17](#_Toc85212191)

[2.1.1.2. Agitadores de turbinas 18](#_Toc85212192)

[2.1.1.3. Agitadores de hélice 18](#_Toc85212193)

[2.1.2.1. Modelo de tanque de mezclado continuo 19](#_Toc85212194)

[2.1.3. Aplicación de tanque de mezclado continuo 22](#_Toc85212195)

[2.2. Caracterización de las plantas industriales con retardo de tiempo. 23](#_Toc85212196)

[2.1.2. Representación del retardo de tiempo en el dominio de la frecuencia 23](#_Toc85212197)

[2.2.2. Aproximaciones polinómicas del retardo de tiempo 25](#_Toc85212198)

[2.2.2.1. Mediante series de Taylor 25](#_Toc85212199)

[2.2.2.2. Aproximaciones de Padé 26](#_Toc85212200)

[2.2.2.3. Mediante polos y ceros múltiples 27](#_Toc85212201)

[2.2.2.4. Otras aproximaciones 27](#_Toc85212202)

[2.2.2.5. Aproximación a modelos de orden reducido 28](#_Toc85212203)

[2.2.3. Identificación del comportamiento dinámico de una planta con retardo de tiempo 29](#_Toc85212204)

[2.3. El predictor de Smith como estrategia de control de plantas con retardo de tiempo de El predictor de Smith (PS) 30](#_Toc85212205)

[2.4. Diseño del controlador 32](#_Toc85212206)

[3.1. Descripción del método 37](#_Toc85212207)

[3.2. Modelamiento matemático 38](#_Toc85212208)

[3.2.1. Diseño e implementación del prototipo 38](#_Toc85212209)

[3.2.2. Descripción del prototipo físico 40](#_Toc85212210)

[3.2.3. Metodología propuesta para la obtención del modelo matemático. 42](#_Toc85212211)

[3.2.4. Validación del modelo matemático. 43](#_Toc85212212)

[3.2.5. Diseño de controlador 44](#_Toc85212213)

[3.2.5.1. Diseño del controlador PI 44](#_Toc85212214)

[3.2.5.2. Diseño del controlador PID 45](#_Toc85212215)

[3.2.5.3. Diseño de un controlador predictor Smith 47](#_Toc85212216)

[3.2.5.4. Diseño de los controladores mediante Matlab 49](#_Toc85212217)

[3.3. Diagrama de bloques estructurales 51](#_Toc85212218)

[3.4. Diagrama de flujos funcional y explicación de cada una de las etapas y su interrelación 52](#_Toc85212219)

[3.5. Pseudocódigo del programa de simulación y descripción de cada etapa de la secuencia del programa. 53](#_Toc85212220)

[3.6. Diseño de los experimentos de validación de resultados (simulación o implementación) Condiciones iniciales, características y definición de parámetros del experimento. 54](#_Toc85212221)

[4.1. Resultados de los experimentos. 56](#_Toc85212222)

[4.2. Tablas resúmenes de los resultados. 59](#_Toc85212223)

[4.3. Gráficos de los resultados. 60](#_Toc85212224)

[4.4. Comparaciones con los resultados de otras técnicas. 62](#_Toc85212225)

[**BIBLIOGRAFÍA** 63](#_Toc85212226)

# **ÍNDICE DE FIGURAS**

**Figura 1.1.** Dinámica de calentamiento de un líquido……………………………….…… 5

**Figura 1.2.** Modelo interno del Predictor Smith……………………………………….…... 6

**Figura 1.3.** Modelo reducido interno del Predictor Smith………………………………... 6

**Figura 2.1.** Mezclador de cilindro…………………………………………………………. 14

**Figura 2.2.** Mezclador de cintas helicoidales. ……………………………………...…… 15

**Figura 2.3.** Mezclador de tornillo sinfín. …………………………………………….…… 15

**Figura 2.4.** Mezclador de paletas…………………………………………………………. 16

**Figura 2.5.** Mezclador de tipo mural………………………………………………………. 16

**Figura 2.6.** Mezclador con estructura móvil……………………………………………....17

**Figura 2.7.** Taladro Mezclador…………………………………………………………......17

**Figura 2.8.** Agitadores de paletas…………………………………………………………. 17

**Figura 2.9.** Agitadores de turbinas………………………………………………………... 18

**Figura 2.10.** Agitadores de hélice…………………………………………………………. 19

**Figura 2.11.** Tanque de mezclado continuo……………………………………………… 19

**Figura 2.12.** Recipientes de agitación sin deflectores y con deflectores……………... 20

**Figura 2.13.** Diseño del tanque agitador…………………………………………………. 20

**Figura 2.14.** Vista de planta, módulo de reactor………………………………………… 21

**Figura 2.15.** Vista de planta, módulo del reactor…………………………….………….. 22

**Figura 2.16.** Tanque de mezclado continuo……………………………………….…….. 23

**Figura 2.17.** Diagrama de fase de tiempo muerto………………………….…………… 24

**Figura 2.18.** Sistema dinámico de un invernadero……………………………………… 31

**Figura 2.19.** Sistema de control PI+……………………………………………………… 33

**Figura 2.20.** Sistema de control PI+PS+Mod Inv……………………………………….. 34

**Figura 2.21.** Sistema de control modificado PI, PI+PS y PI+PS+Mod Inv…………… 34

**Figura 2.22.** Resultados de la simulación de las señales de erro en los sistemas de control con estructura clásica y estructura modificada del Predictor Smith……….….. 35

**Figura 3.1.** Metodología de diagnóstico del proyecto…………………………………... 37

**Figura 3.2.** Mezclador o agitador para tanques de dosificación…………………..…… 38

**Figura 3.3.** Sensor de temperatura DS18B20…………………………………...…….... 38

**Figura 3.4.** Válvula de flujo de fabricación propia………………………………..……... 39

**Figura 3.5.** Esquema del sistema físico………………………………………………….. 40

**Figura 3.6.** Respuesta del modelo a una señal de entrada tipo escalón y un disturbio tipo escalón…………………………………………………………………………………... 43

**Figura 3.7.** Diagrama de bloques del sistema a lazo cerrado usando un controlador PI. ………………………………………………………………………………………………… 44

**Figura 3.8.** Diagrama de bloques del sistema a lazo cerrado usando un controlador PID……………………………………………………………………………………………. 45

**Figura 3.9.** Diagrama de bloques del sistema a lazo cerrado usando un controlador predictor Smith. ………………………………………………………………………………………………… 47

**Figura 3.10.** Diagrama de bloques del sistema a lazo cerrado usando un controlador predictor Smith………………………………………………………………………………. 48

**Figura 3.11.** Diagrama de bloques del sistema a lazo cerrado usando un controlador predictor Smith………………………………………………………………………………. 51

**Figura 3.12.** Simplificación del sistema…………………………………………….…………………………………………. 51

**Figura 3.13.** Diagrama de flujo funcional……………………………………………………………………………………… 52

**Figura 3.14.** Definición de tiempos de respuesta y estabilización………………………………………………………………………………… 55

**Figura 4.1.** Respuestas del sistema de control de los tres diferentes tipos de controladores sintonizados con un $T\_{C}=2L$. ……………………………………………... 57

**Figura 4.2.** Respuestas del sistema de control de los tres diferentes tipos de controladores sintonizados con un $T\_{C}=L$. ………………………………………………. 58

**Figura 4.3.** Respuestas del sistema de control de los tres diferentes tipos de controladores sintonizados con un $T\_{C}=L/2$. ………………………………………….… 58

**Figura 4.4.** Respuestas del sistema de control de los tres diferentes tipos de controladores sintonizados con un $T\_{C}=L/4$. ……………………………………….…… 59

**Figura 4.5.** Exposición de los parámetros de performance en las respuestas de los sistemas de control con diferentes controladores sintonizados con un $T\_{C}=2L$……... 60

**Figura 4.6.** Exposición de los parámetros de performance en las respuestas de los sistemas de control con diferentes controladores sintonizados con un $T\_{C}=L$………. 61

**Figura 4.7.** Exposición de los parámetros de performance en las respuestas de los sistemas de control con diferentes controladores sintonizados con un $T\_{C}=L/2$……. 61

**Figura 4.8.** Exposición de los parámetros de performance en las respuestas de los sistemas de control con diferentes controladores sintonizados con un $T\_{C}=L/4$……. 62

# **ÍNDICE DE TABLAS**

**Tabla 3.1**. Parámetros que intervienen en el sistema…………………………………... 41

**Tabla 3.2.** Criterios de aceptación de respuesta de tensión…………………………… 55

**Tabla 3.3.** Descripción de casos del experimento………………………………………. 55

**Tabla 4.1.** Parámetros de performance para el análisis de las respuestas de salida.. 59

**AGRADECIMIENTOS**

# **RESUMEN**

La presente tesis Diseño e implementación del control de temperatura para tanques de mezclado continúo basado en un predictor Smith, presenta 2 partes importantes, la implementación del módulo y la validación del algoritmo de control basado en el Predictor Smith

El diseño del módulo se realizó teniendo en cuenta la necesidad de los estudiantes y docentes para realizar prácticas en tópicos de control y automatización.

Con respecto al diseño del controlador se tomó como referencia el Predictor Smith y sus variaciones debido a su utilidad frente a sistemas con retardo como lo es el sistema de control de temperatura para tanques de agitación continua. El algoritmo de control será comparado con el clásico controlador PID tan usado en la industria.

**Palabas clave**

Predictor Smith, prototipo, sistema de control, tanques de mezclado

# **INTRODUCCIÓN**

La mayoría de los procesos industriales presentan retardos en sus dinámicas que dificultan la tarea de regulación de sus variables dichos retardos pueden ser intrínseco al sistema o puede ser introducido por el diseño del controlador.

La presente tesis desarrolla el diseño de un controlador basado en una variante del predictor Smith aplicado a un sistema tanque de agitación continua para el control de temperatura.

La idea se basa en que, al conocer el retardo, es posible saber qué es lo que sucederá luego del mismo, es decir, podemos predecir el comportamiento del proceso.

En este trabajo de investigación aprenderemos en detalle cómo diseñar e implementar el predictor de Smith en un tanque de mezclado industrial para luego comparar su desempeño con el clásico controlador PID.

La principal ventaja de este controlador consiste en que elimina el retardo de tiempo de la ecuación característica del sistema de control en lazo cerrado, de este modo, el problema de diseño e implementación de sistemas de control de procesos con retardo de tiempo se realiza como si no existiera dicho retardo.

# **CAPÍTULO I**

# **ASPECTOS INTRODUCTORIOS**

# **Antecedentes bibliográficos**

Veronesi [38] Estudia el impacto que tiene el tiempo de retardo del modelo de la planta en la estrategia de control Predictor de Smith. El estudio se realiza en una cañería de gran distancia por donde circula un líquido. El retardo es producido debido a que el sensor de flujo se encuentra al otro extremo de la válvula de flujo.

El estudio afirma que si el retardo del modelo es mal estimado el Predictor responde de manera inapropiada disminuyendo drásticamente su desempeño e inclusive puede tornar el sistema inestable, llegando a tener peores resultados que un control PID simple, sin embargo, si el retardo del modelo es similar al de la planta se obtiene la compensación deseada. Es por ello por lo que se hacen pruebas con un controlador PID con Predictor de Smith, variando el retardo en un ±50% del retardo real.

Si el retardo es sobreestimado el sistema responde ante una entrada escalón de forma lenta, llegando al setpoint en un tiempo elevado, pero sin tener un sobrepaso: Por el caso contrario si el retardo es subestimado, el sistema responde forma brusca y rápida, generando un sobrepaso considerable para luego estabilizarse en el setpoint. Es por ello por lo que se calcula el retardo del proceso mediante un algoritmo en tiempo real, el cual se implementa en el controlador, logrando ajustar el retardo incluso si hay cambios en el proceso.

Palmor y Blau [27] implementan la estrategia Predictor de Smith con un PI autosintonizante, enfocado en un proceso de primer orden con retardo. Posteriormente se estima el modelo del proceso mediante mínimos cuadrados.

Además, se afirma que la estrategia de control de Predictor de Smith obtiene mejores resultados que un PI o PID en los procesos industriales, inclusive si el modelo de la planta usado en el compensador del retardo es aproximado a un proceso orden mucho menor que el proceso real.

Finalmente se sugiere una forma de sintonizar un P.S de una forma óptima y simple, priorizando el retardo como principal incertidumbre, por lo que los parámetros son calculados en función a la variación del retardo estimado.

Benites-González, Rivas-Pérez y Feliu-Batlle [17] desarrolla el diseño de un controlador con estructura modificada del predictor Smith para el control efectivo de la concentración de la mezcla en el proceso de producción de medicamentos inyectables en la industria farmacéutica su propuesta novedosa, su propuesta fue la introducción de un bloque adaptativo basado en lógica difusa para la estimación y reajuste del retardo de tiempo así como también la incorporación de un compensador anticipatorio FF(s) para rechazar el efecto de las perturbaciones externas medibles, demostrando que el controlador diseñado posibilita controlar con elevada efectividad dicha planta considerando diferentes escenarios reales de operación industrial.

Dong, Yonhong y Gaohong [10] diseñan un sistema de control de temperatura del agua de una piscina interior basados en el predictor Smith y un algoritmo de optimización automática difusa, se reconoce que el sistema de control PLC tradicional basado en el controlador PID no puede cumplir con los requisitos de control, los experimentos y las aplicaciones mostraron que el esquema de control propuesto soluciono el grave retraso y el sistema de inercia serio pudo lograr un buen efecto de control

Khodadadi y Dehghani [18] diseña un controlador PID de autoajuste de lógica difusa basado en un predictor Smith para un sistema de calefacción, el predictor de Smith se propone como una solución para controlar el tiempo de retrasado. Además, para superar la condición incierta del modelo, se emplea un controlador PID de autoajuste basado en la lógica difusa, se resalta la capacidad de esa estructura para superar la incertidumbre y el cambio en parámetros del sistema, los hallazgos obtenidos de varios resultados de simulación verificaron la notable precisión del método propuesto en el control del sistema de calefacción.

Thuengsripan y Suksri [37] tratan del diseño de un controlador de temperatura basado en un predictor Smith mediante el método de diagrama de coeficientes (CDM), advierten de lo difícil que es encontrar un modelo matemático preciso del sistema práctico y muy sensible a sistemas inciertos con retardo de tiempo variable para el predictor Smith, los resultados de la simulación muestran que el sistema de control con el diseño del predictor Smith de CDM es estable y robusto, al tiempo que proporciona el rendimiento deseado del sistema en el dominio del tiempo.

[Feliu-Batlle](https://link.springer.com/article/10.1007/s12555-012-0355-z#auth-Vicente-Feliu_Batlle) , [Rivas-Pérez](https://link.springer.com/article/10.1007/s12555-012-0355-z#auth-Raul-Rivas_Perez) y [Castillo-García](https://link.springer.com/article/10.1007/s12555-012-0355-z#auth-Fernando_J_-Castillo_Garc_a) [11] propone el diseño de un controlador simple de orden fraccional combinado con un esquema de predicción de Smith para controlar la temperatura de un horno de recalentamiento de planchas de acero, comparan el rendimiento del controlador de orden fraccional propuesto con un controlador PI estándar , también combinado con un predictor Smith, un controlador LQR y un  controlador H ∞ robusto, utilizando cuatro índices de rendimiento: tres relacionados con el rendimiento de salida (tiempo de establecimiento, sobre impulso y error absoluto integral), y un cuarto relacionado con el esfuerzo de control ( TV ). El análisis de estos índices muestra que el controlador de orden fraccional simple proporciona valores más bajos de los índices comparados cuando el retardo de tiempo se vuelve mucho mayor que el valor nominal.

Jacobs y Chen [3] donde reportan la implementación de un control óptimo y un estimador de estado utilizando como base un modelo algebraico donde la masa de Zinc depositada sobre la lámina depende de la distancia de las cuchillas de aire a la lámina, de la velocidad de la lámina, de la presión de aire proporcionada por las cuchillas y de una constante de proporcionalidad y obtienen la versión discreta para la realización de su análisis.

Chiasson y Lee [24] donde proponen la asignación de coeficientes en base al uso de compensadores dinámicos para sistemas con retardo (una entrada) con los reguladores tradicionales. Por lo que se requiere de estrategias que permitan adaptarse a estas variaciones ante distintas condiciones de operación, lo que demanda métodos de sintonía que aseguren que la estabilidad y las especificaciones temporales se mantengan en todo el rango de operación.

# **Descripción del problema**

El control de temperatura en un tanque de agitación continua en un proceso industrial que tradicionalmente utiliza el algoritmo PID, sin embargo, ante retardos se inestabiliza, además no toma en cuenta la optimización del recurso energético de los actuadores.

# **¿Qué es un Retardo?**

El retardo es un fenómeno que transcurre por el desplazamiento temporal que puede aparecer entre dos o más variables de control y este puede ser generado por ejemplo por el tiempo necesario para transportar masa, energía o información.

En el siguiente ejemplo se muestra como el retardo o el tiempo muerto aparece en la dinámica de calentamiento de un líquido al interior de un tanque.

Cuando la resistencia se activa (escalón azul) observemos que la temperatura al final del tubo solo responde un tiempo después (línea roja), sin embargo, si tuviéramos la posibilidad de colocar un sensor justo en el tanque, el retardo de tiempo sería eliminado (línea azul).

![Imagen que contiene Diagrama  Descripción generada automáticamente](data:image/png;base64...)

**Figura 1.1.** Dinámica de calentamiento de un líquido. Fuente: Referencia [7]

# **Sistemas de Control para procesos con Retardo**

Para tratar el problema de los retardos sobre los sistemas de control en lazo cerrado existen dos grandes líneas de investigación las cuales son:

Compensadores de tiempo muerto (DTC)

Control predictivo basado en modelo (MPC)

En 1957, el norte americano **Otto Smith** sugirió un compensador que remueve efectivamente el retardo de la ecuación característica del sistema. Ese compensador de tiempo muerto fue conocido como el **predictor de Smith** (PS) y fue la base de diversos estudios que actualmente se vienen desarrollando para tratar los problemas del retardo dentro de los lazos de control.

# **Predictor de Smith**

La estructura del predictor de Smith (*Smith predictor*) viene dado por la siguiente representación de modelo interno:

![Diagrama, Esquemático  Descripción generada automáticamente](data:image/png;base64...)

**Figura 1.2.** Modelo interno del Predictor Smith.

Fuente: Referencia [38]

En la estructura el control primario es *C(s)*, el proceso real es *P(s)*, el modelo rápido o modelo sin retardo es *Gn(s)* y el retardo viene dado por la expresión $e^{-L\_{n}s}$.

Note además que si el modelo es igual a el proceso real implica que $P\_{n}\left(s\right)=G\_{n}\left(s\right)e^{-L\_{n}s}=P(s)$.

Este compensador de tiempo muerto es capaz de predecir la salida del proceso real, y(t), por medio de la dinámica sin retardo $G\_{n}$, o sea que con esa estructura el control es capaz de predecir el comportamiento del proceso real, P(s), es un tiempo igual al retardo Ln.

El método de Smith en Control se puede reducir utilizando el algebra de bloques a la siguiente representación equivalente:

![Diagrama  Descripción generada automáticamente](data:image/png;base64...)

**Figura 1.3.** Modelo reducido interno del Predictor Smith.

Fuente: Referencia [38]

Donde el control equivalente viene dado por:

$C\_{eq}=\frac{C(s)}{1+C\left(s\right)G\_{n}\left(s\right)-C\left(s\right)P\_{n}(s)}$ 1.1

Así las relaciones de la entrada r(t) y la salida y(t) suponiendo $P\left(s\right)=P\_{n}(s)$ viene dado por:

$H\_{yr}\left(s\right)=\frac{y(t)}{r(t)}=\frac{C\left(s\right)P\_{n}(s)}{1+C\left(s\right)G\_{n}(s)}$ 1.2

Notamos que, gracias a la estructura del predictor de Smith, la ecuación característica del proceso queda libre de la influencia del retardo.

Por otro lado, la relación de la entrada q(t) y la salida y(t) viene dado por:

$H\_{yq}\left(s\right)=\frac{y(t)}{q(t)}=P\_{n}\left(s\right)\left[1-\frac{C\left(s\right)P\_{n}\left(s\right)F\_{r}\left(s\right)}{1+C\left(s\right)G\_{n}\left(s\right)}\right]$ 1.3

# **Limitaciones del Predictor de Smith**

El predictor de Smith tiene algunas limitaciones las cuales pueden enunciarse a continuación:

El predictor de Smith solo sirve para procesos estables.

La estructura NO es capaz de acelerar la dinámica de rechazo de perturbaciones.

No se puede usar la estructura del predictor de Smith en procesos integradores o en procesos inestables porque la estructura es internamente inestable, lo que quiere decir que, si entra una perturbación, la estructura se inestabilizara para estos dos procesos.

Pequeños errores de modelo, por ejemplo, cuando P(s) es diferente a Pn(s), puede hacer que la estructura entre rápidamente a la inestabilidad.

Estas limitaciones del predictor de Smith pueden ser tratadas por medio de una modificación propuesta por Normey-Rico y Camacho en 1997, adicionando a la estructura un filtro. Esa nueva estructura se conoce como el**Predictor de Smith Filtrado**. Esa estructura será estudiada en otra entrada.

# **Formulación del problema**

El presente trabajo de investigación busca dar respuesta a la siguiente pregunta: ¿Mediante el uso del modelo de predictor Smith es posible construir, implementar y validar el algoritmo de control de temperatura en tanques de mezclado continuo?

# **Justificación e importancia de la investigación**

El presente trabajo de investigación posee justificación practica ya que con el diseño y la implementación de un módulo de control de temperatura para un tanque de mezclado continuo usando un predictor Smith, se estaría contribuyendo a la disminución o merma de un fenómeno muy frecuente en los sistemas dinámicos, el retardo de tiempo al medir la temperatura, el cual genera efectos perjudiciales como la detección tardía de perturbaciones y la disminución de los márgenes de fase y ganancia, afectando la estabilidad global del sistema.

Así mismo tiene justificación económica y técnica pues la implementación de un módulo de control de temperatura en un tanque de mezclado continúo usando un predictor Smith, ayudara a tener un control más eficiente de la variable temperatura en los tanques de mezclado continuo, lo cual se traduciría en una mejora en la calidad de los procesos.

# **Objetivos**

# **Objetivo General**

Diseñar e implementar un módulo de control de temperatura para un tanque de mezclado continúo usando un predictor Smith

# **Objetivos Específicos**

* Construir el modelo de control en donde se llevará a cabo el proceso.
* Implementar el algoritmo de control ya que será el medio para realizar modificaciones orientadas a la optimización.
* Validar el algoritmo de control para ver su desempeño en un tanque de mezclado continúo comparándolo con el controlador PID.

# **Hipótesis**

# **Hipótesis principal**

El módulo diseñado que usa el predictor Smith es factible para el control de temperatura aplicado a un tanque mezclador continuo.

# **Variables e indicadores**

# **Variable independiente**

El uso de un predictor Smith en el diseño e implementación de un módulo de control temperatura.

# **Variable dependiente**

Control de la temperatura de un tanque de mezclado continuo.

# **Indicadores**

Representación discreta de la estructura del predictor Smith.

Modelo de control del proceso.

Creación del algoritmo de control.

Predicción del comportamiento del proceso.

Complejidad del modelo dinámico del proceso.

Eliminación de retardos de tiempo.

# **Unidad de análisis**

Se plantea implementar el predictor Smith en tanques de mezclado de plantas industriales inestables debido al retardo de tiempo, las cuales debemos tener en cuenta en las etapas de análisis como en las de diseño de los controladores.

# **Tipo y nivel de Investigación**

La investigación es de tipo experimental y aplicada, mediante el análisis de las variables y simulaciones del predictor Smith en procesos a nivel industrial para poner a prueba las estrategias de control de temperatura implementadas en tanques de mezclado continúo.

La investigación es de nivel de Maestría en Ciencias, ya que involucra diversos conocimientos y experiencias, tales como:

• Conocimientos en modelos matemáticos y experiencias en simulación de procesos de mezclado en equipos de alta tensión.

• Conocimientos y manejos sobre los diversos tipos de controladores predictivos que existen.

• Conocimientos sobre las ventajas y desventajas que conlleva el uso del predictor Smith.

# **Periodo de análisis:**

Los tiempos de retardo o tiempos muertos constituyen una parte fundamental de la dinámica de muchos procesos industriales, siendo una limitante para conseguir un control adecuado; por ello, es importante tomarlo en cuenta durante el diseño del controlador del proceso ya que son una fuente de inestabilidad en los lazos de control.

El tiempo muerto, es el que transcurre desde el instante en que se produce un cambio en una variable de entrada al proceso hasta el instante en que el efecto de dicha variación comienza a observarse en la variable de salida.

En nuestro caso, se debe al tiempo que tarda un fluido en circular de un punto a otro a través de tuberías hacia el tanque de mezcla continua o al sistema de medida de la variable controlada, considerando también el tiempo que el predictor Smith toma en recibir las señales.

Por lo tanto, el periodo de análisis es la suma de los retardos percibidos gracias a las tuberías por donde pasa el fluido y la suma de tiempo de reacción de los instrumentos de medición.

# **Fuentes de información e instrumentos utilizados:**

# **Caudalímetro**

Es un dispositivo que se utiliza para medir el caudal o la cantidad de un fluido que circula en un punto, ya sea en grande o en pequeñas cantidades. El uso del caudalímetro se aplica principalmente a la medición y contabilización de fluido que circula a través de un conducto dentro de un proceso industrial.

# **Predictor Smith**

Propuesto en 1957 por el norteamericano Otto Smith es desde lejos el compensador de tiempo muerto más utilizado en el control de plantas con retardo de tiempo debido a su gran efectividad y simple implementación. Esta estructura de control surgió con la idea de mejorar el desempeño de los controladores clásicos (PI o PID) en el control de plantas con retardo de tiempo dominante.

La principal ventaja radica en la eliminación del retardo de tiempo de la ecuación característica del sistema de control en lazo cerrado, cuando su modelo interno describe de forma perfecta el comportamiento dinámico de la planta. Sin embargo, esta estructura de control en su versión inicial presenta limitaciones, entre las que se destacan:

- No puede aplicarse en el control de plantas con integradores, inestables, o con retardo de tiempo variante en el tiempo.

- Pobre desempeño frente a incertidumbres en el modelado de las plantas

- Baja robustez frente a perturbaciones externas medibles o no medibles.

# **Matlab**

La modelización es una forma de crear una representación virtual de un sistema real que incluye software y hardware. Si los componentes de software de este modelo están gobernados por relaciones matemáticas, es posible simular esta representación virtual con una amplia gama de condiciones para ver cómo se comporta.

La modelización y la simulación son especialmente útiles para probar condiciones que podrían resultar difíciles de reproducir solamente con prototipos de hardware, especialmente en la primera fase del proceso de diseño, cuando es posible que no esté disponible el hardware. La iteración entre la modelización y la simulación puede mejorar la calidad del diseño del sistema en una etapa temprana y reducir así el número de errores descubiertos más adelante en el proceso de diseño.

# **Técnicas de recolección y procesamiento de datos**

La recopilación de datos es el proceso de recolección y medición de información sobre las variables seleccionadas de forma sistemática y establecida, que permite responder a las preguntas pertinentes y evaluar los resultados. Ayuda a científicos y analistas a recolectar los puntos principales como información recopilada. Aunque los métodos varían según la disciplina, el énfasis en la recolección exacta y honesta sigue siendo el mismo. El objetivo de toda la recolección de datos es capturar evidencia de calidad que luego se traduce en análisis de datos y permite construir una respuesta convincente y creíble a las preguntas que se han planteado.

Como parte del diseño metodológico es necesario establecer el procedimiento de recolección de datos, el tipo de variable, la exactitud elemental, el punto de recolección y la fuente de información. Los vínculos entre una variable, su origen y los procedimientos prácticos para su colección tienen la posibilidad de contribuir a seleccionar procedimientos apropiados. Los principales métodos de recopilación de datos son:

* Registros: los registros y licencias son particularmente valiosos para los censos completos, pero se limitan a variables que cambian lentamente.
* Cuestionarios: formularios que los encuestados devuelven cumplimentados. Un método poco costoso que resulta útil cuando los índices de alfabetización son altos y los encuestados colaboran.
* Entrevistas: formularios que se cumplimentan a lo largo de una entrevista con el encuestado. Más caros que los cuestionarios, pero mejores para preguntas más complejas, y cuando se dan unos índices de alfabetización bajos o se encuentra menos colaboración.
* Observaciones directas: la realización de mediciones directas es el método más preciso para todas las variables.
* Presentación de informes: la principal alternativa a la realización de mediciones directas. La preparación de informes presupone la alfabetización y requiere espíritu de colaboración, pero ello puede reforzarse mediante una obligación legal y mediciones directas

El objetivo de cualquier ciencia es adquirir conocimientos y elegir el método más adecuado que permita conocer la realidad. Respecto a los métodos de investigación están muy relacionados con los instrumentos de recolección, entre los investigadores sociales existe la disyuntiva entre usar métodos cuantitativos o cualitativos; sin embargo, en un trabajo la cuestión cuantificable no tiene por qué ser opuesta a la cualitativa, según [31]; sin embargo, pueden ser complementarios y existir una investigación integrada cuantitativa y cualitativamente. [6]

Con respecto al procesamiento de datos, gracias a la automatización y digitalización de la recolección de datos en tiempo real, las tecnologías de software y hardware están totalmente listas y desarrolladas para ser implantadas. Con el uso de un sistema de análisis de datos en tiempo real se puede agregar en un solo sistema toda la información.

Por ejemplo, tecnologías como Big Data, Business Intelligence, Analytics, Cloud Computing, dispositivos Wearables, etiquetas RFID, etc. Por otra parte, existen distintos protocolos para la transmisión de los datos. Los más conocidos son: MQTT, MQTT-S, CoAP, REST-API y XMPP. [32]

Por lo mencionado anteriormente, el acceso a la información se torna mucho más fácil y reduce el tiempo invertido previamente para recopilar información procedente de diversas fuentes y derivarlos a cualquiera de los numerosos gestores de bases de datos relacionales tales como Microsoft SQL Server, Paradox, Oracle, Sybase, MySQL, entre otros. [32]

Una vez recopilados y almacenados los resultados se debe generar un informe de estos, con el fin de realizar un filtro de los datos obtenidos para estudiar puntos específicos y no todos los resultados de cada uno de los puntos en los cuales se tomaron medidas. Para este análisis se debe aplicar diferentes herramientas o métodos que ayuden en cuanto al control y gestión de las variables estudiadas. [35]

# **Garantizar una buena gestión de datos**

Una buena gestión de datos implica desarrollar procesos eficaces para la recolección y el registro sistemáticos de datos, almacenamiento seguro, depuración, transferencia (por ejemplo, entre distintos tipos de programas informáticos utilizados para el análisis), la presentación eficaz y la accesibilidad de los datos para su verificación y utilización por terceros. Los aspectos de calidad de los datos que se mencionan comúnmente son:

* Validez: los datos miden lo que se pretende que midan.
* Fiabilidad: los datos se miden y recopilan sistemáticamente según las definiciones y metodologías estándar; los resultados son los mismos cuando se repiten las mediciones.
* Exhaustividad: se incluyen todos los elementos de información (según las definiciones y metodologías especificadas).
* Precisión: los datos están lo suficientemente detallados
* Integridad: los datos están protegidos de sesgos o manipulaciones deliberados por motivos políticos o personales.
* Puntualidad: los datos están actualizados y la información está disponible a tiempo. Es recomendable utilizar herramientas normalizadas de recolección de datos, que ya se hayan probado en situaciones reales, y mejorarlas en caso necesario para maximizar la calidad de los datos. [29]

# **CAPÍTULO II**

# **MARCO TEÓRICO Y CONCEPTUAL**

# **Características generales de los tanques industriales**

1. **Mezcladores Móviles**

Como se muestra en la imagen 1 consiste en una carcasa de diferentes geometrías ya sean cilíndricos, cúbicos o cónicos. La carcasa va montada sobre un eje que al rotar sobre este se genera el mezclado. En la parte superior de la carcasa lleva una compuerta que sirve para el ingreso de las materias primas y por acción del volcamiento y la gravedad, el producto terminado salga a los recipientes para su dosificación. La velocidad de mezclado oscila entre los 300 y 1000 rpm dependiendo el tamaño, la forma del mezclador y el tipo de materias primas que se van a mezclar.[19]

![Imagen que contiene interior, plata, tabla, monitor  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.1.** Mezclador de cilindro.

Fuente: Referencia [31]

1. **Mezcladores de carcaza estacionaria**

Son equipos donde la carcasa permanece estática, en su interior poseen una serie de elementos que ejecutan el mezclado como aire a chorro, cuchillas, tornillos o paletas.

1. **Mezcladores de cintas helicoidales**

Como se muestra en la figura poseen una estructura central cilíndrica, dentro de la cual se encuentra un agitador de cintas, por lo general poseen dos cintas helicoidales en sentido contrario que están montadas sobre un eje que les dará el movimiento. La cinta externa produce un movimiento axial, y la segunda genera un movimiento radial. Al generarse estos movimientos se produce una turbulencia que lleva al mezclado de las materias primas.

![Imagen que contiene hombre, sartén, aire  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.2.** Mezclador de cintas helicoidales.

Fuente: Referencia [31]

1. **Mezclador de tornillo sinfín vertical**

![Imagen que contiene tabla  Descripción generada automáticamente](data:image/png;base64...)Consiste en una carcasa cónica con un tornillo sinfín en su interior que transporta las materias primas hacia la parte superior al generarse la rotación del sinfín las materias primas se mezclan y por acción de la gravedad caen a la parte inferior de la carcasa. Los mecanismos de mezcla son por convección debido al movimiento planetario y por difusión debido al sinfín. Posee una compuerta en la parte inferior para la salida del producto terminado.

Figura 2.3. Mezclador de tornillo sinfín.

Fuente: Referencia [31]

1. **Mezclador de paletas**

Como se muestra en la imagen 4 consiste en un recipiente cilíndrico vertical donde las materias primas son mezcladas por la acción de palas o paletas unidas a un eje rotatorio central.

![Diagrama  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.4.** Mezclador de paletas.

Fuente: Referencia [31]

1. **Mezclador de tipo manual**

Es un agitador que se encuentra fijo en la pared, consta de una moto reductora y un cilindro hidráulico que produce el movimiento del cabezal.

![Diagrama  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.5.** Mezclador de tipo mural.

Fuente: Referencia [31]

1. **Mezclador con estructura móvil**

Es del mismo tipo del mezclador tipo mural su gran diferencia radica en la estructura que consta de unas ruedas para su fácil desplazamiento.

![Diagrama  Descripción generada automáticamente](data:image/png;base64...)

Figura 2.6. Mezclador con estructura móvil.

Fuente: Referencia [31]

1. **Taladro mecánico**

Es un taladro industrial al que por lo general se acopla el eje de mezclado, el acople se realiza como una broca normal.

![](data:image/png;base64...)

**Figura 2.7.** Taladro Mezclador

Fuente: Referencia [31]

# **Tipos de tanques de mezclado**

Los agitadores o propelas son las piezas que van montadas en la parte inferior del eje que se va a utilizar para el mezclado, estos agitadores se utilizan para lograr una mejor homogenización de las materias primas, en la actualidad existen tres tipos de agitadores: Paletas, Turbina, Hélice.[30]

# **Agitadores de paletas**

Es un agitador radial, útil para mezclas simples, por ejemplo, mezcla de líquidos o disolución de sólidos. Produce agitación suave y no necesita de placas deflectoras.

![](data:image/png;base64...)

(a) Pala plana (b) Agitador de reja

![](data:image/png;base64...)

(c) Agitador de ancla (d) Palas de giro opuesto

**Figura 2.8.** Agitadores de paletas

Fuente: Referencia [31]

# **Agitadores de turbinas**

Lo forma un impulsor con más de cuatro hojas, se parece a agitadores de múltiples, se usan para un amplio intervalo de viscosidades.

(d) Discos con aspas

![](data:image/png;base64...)

(a) Hoja sencilla (b) Hoja con resalte (c) Hoja curva

**Figura 2.9.** Agitadores de turbinas

Fuente: Referencia [31]

# **Agitadores de hélice**

Formado por elementos impulsores de hojas cortas, estos giran a toda la velocidad del motor”.

El flujo se mueve en una dirección de acuerdo a los requerimientos solicitados para luego ser desviadas.

La componente radial y la longitudinal ayudan a la mezcla, pero no siempre la componente rotatoria”.[31]

![Diagrama, Texto  Descripción generada automáticamente con confianza media](data:image/png;base64...)

**Figura 2.10.** Agitadores de hélice

Fuente: Referencia [31]

# **Modelo de tanque de mezclado continuo**

El proceso de un tanque consta de agregar dos fluidos uno a temperatura baja y otro a una gran temperatura. Asumiendo que las diversas propiedades de los fluidos son uniformes. La sección de una tubería que va desde el tanque hasta llegar dispositivo de temperatura es la parte más relevante del proceso y el cual es el que genera más retrasos en el transporte de las propiedades del fluido.[26]

![Diagrama, Esquemático  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.11.** Tanque de mezclado continuo

Fuente: Referencia [26]

La agitación es una acción mecánica que realiza movimientos fuertes y anómalos en el seno de la masa fluida. Los tanques agitados se han usados crucialmente en la industria de alimentos, química y biotecnología. Los agitadores se dividen en dos los radiales que son los que producen corrientes paralelas al eje del impulsor y los axiales que producen corrientes en dirección radial tangencial.[8]

![Diagrama, Dibujo de ingeniería  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.12.** Recipientes de agitación sin deflectores y con deflectores

Fuente: Referencia [8]

![Diagrama  Descripción generada automáticamente](data:image/png;base64...)Los recipientes sin deflectores que se agiten con un agitador axial tendrán una predisposición a que se forme un flujo de remolino y en los recipientes con deflectores se impide esa rotación con 4 deflactores.[8]

![Diagrama  Descripción generada automáticamente](data:image/png;base64...)![Diagrama  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.13.** Diseño del tanque agitador

Fuente: Referencia [8]

Para el volumen se utiliza datos del hidrolisis alcalino del acetato de etilo.

El valor de la constante de velocidad a 26ºC es de 2,7843 L/mol\*min y el orden global de la reacción es de 2,24, la concentración inicial bombeada es de 0,1 mol/L con un caudal de 0,1 L/min y una conversión de 0,35.

$V=\frac{Fa\_{0}-fa}{-ra}$ 2.1

![](data:image/png;base64...) 2.2

![Imagen que contiene Texto  Descripción generada automáticamente](data:image/png;base64...) ![Imagen que contiene Diagrama  Descripción generada automáticamente](data:image/png;base64...)

V=0.625 L debido a que tendrá un agitador será un 20% adicional V=0.750 L. [9]

![](data:image/png;base64...) 2.3

![](data:image/png;base64...)

Ahora veremos los planos del diseño del reactor, sistema de agitación y módulo de operación.

![](data:image/png;base64...)

**Figura 2.14.** Vista de planta, módulo de reactor

Fuente: Referencia [24]

![Imagen que contiene interior, pequeño, agua, parado  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.15.** Vista de planta, módulo del reactor

Fuente: Referencia [9]

# **Aplicación de tanque de mezclado continuo**

El tanque de mezclado continuo o tanque con agitación continua (CSTR) es uno de los más usados en la industria química debido a que presenta ciertas ventajas que se derivan de la uniformidad de presión, composición y temperatura. Una de ellas es la posibilidad de ser operados en condiciones isotérmicas, aun cuando el calor de reacción sea alto. Esta característica es aprovechada cuando se desea que el reactor opere en intervalos pequeños de temperatura para reducir las reacciones secundarias que podrían degradar al producto o para evitar velocidades desfavorables.

Los reactores tipo CSTR se utilizan preferentemente en sistemas de fase líquida a presiones bajas o medias. Pueden usarse cuando el calor de reacción es alto, pero sólo si el nivel de temperatura en la operación isotérmica es adecuado desde otros puntos de vista del proceso (como, por ejemplo, que la temperatura no sea tan alta que ponga en riesgo la seguridad del reactor). También pueden emplearse para reacciones altamente exotérmicas y con altas velocidades de reacción, en cuyo caso se puede ajustar la velocidad de la alimentación y el volumen del reactor (etapa de diseño) a fin de eliminar el calor necesario para que la masa reaccionante se mantenga dentro los valores de temperatura permitidos.[28]

La mayoría de las aplicaciones son de índole químicas y para investigaciones de los productos de las distintas reacciones que se pueden hacer. Una aplicación que se ha estado investigando es la de hacer el proceso de biooxidación de minerales para la extracción de oro y plata. El proceso consta de la biooxidación de forma continua y discontinua, esto permitirá conocer los niveles de oxidación de los sulfuros y luego un proceso de cianuración para la obtención de los metales oro y plata. Existe una investigación de la mina Zancudo en Colombia. La cual su resultado fue que mejoraron la recuperación de oro de 40 % (blanco) a 79 %, mientras que la plata aumenta de 51 % a 80 %. A partir de estos resultados se consideró que la biooxidación de sulfuros del mineral de la mina el Zancudo, en un reactor de tanque agitado, se torna en una opción prometedora como pretratamiento oxidante antes del proceso de cianuración.[4]

Otro de los usos más comunes que se dan es para el tratamiento de aguas residuales. El tanque agitado continuo forma parte de los diferentes equipos que se usan para este fin. El agua residual es transportada al tanque agitado continuo, en este tanque los microorganismos anaerobios degradan las largas cadenas orgánicas en cadenas más cortas. En el clarificador secundario la biomasa del tanque agitado es separada del agua, esta biomasa es bombeada de vuelta al tanque agitado continuo.[14]

![Diagrama  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.16.** Tanque de mezclado continuo

Fuente: Referencia [14]

# **2.2. Caracterización de las plantas industriales con retardo de tiempo.**

# **Representación del retardo de tiempo en el dominio de la frecuencia**

Estimamos el modelo lineal de un tiempo muerto puro L, establecido por:

$G (s)= e -Ls con L>0$ 2.4

La frecuencia se obtiene calculando

$G (jw) = e -jwL para w \in R, w>0$ 2.5

La ganancia y atraso de fase del sistema G(s) son establecidas por:

$G (jw) |= | e -jw |= 1$ 2.6

$φ G (jw) = φ e -jLw= -wL para todo w>0$ 2.7

La magnitud de la ganancia es igual a uno, el diagrama de fase es afectado por el tiempo muerto.

![Gráfico, Gráfico de líneas  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.17.** Diagrama de fase de tiempo muerto

Fuente: Referencia [5]

Ondas senoidales desfasadas por un tiempo L=1s

En la figura que se muestra una onda senoidal que está a un periodo Tb y a una señal con un retraso de L seg. Donde se observa que la amplitud de esa señal retrasada es igual a la amplitud de la señal principal, por ende, la ganancia de todas las frecuencias es 1.

Esta fase puede ser calculada así:

$φ= \frac{-L}{Tb}2π = -wpL.$ 2.8

La frecuencia normalizada dada por (𝑤𝑛):

$$wn= wL$$

Donde w en rad/s, L en segundos, wn en radianes.

En la figura mostrada se modela un diagrama de fase del factor muerto.

En esa figura se muestra el valor de la fase para distintas frecuencias.

Se nota que φ= -𝑤𝑛; es una relación lineal y también tiene una forma exponencial en la escala de sus gráficos.[12]

# **Aproximaciones polinómicas del retardo de tiempo**

Las aproximaciones polinómicas se utilizan para realizar estudios de desempeño de la estabilidad y diseño del sistema de control. Para ello es necesario aproximar el tiempo de retardo por alguna función racional s, usualmente mediante el cociente de polinomios.[2]

Se definirá primero la función tiempo muerto como:

$ f\_{0}\left(s\right)=e^{-t\_{m}s}$2.9

Y se establecerán las aproximaciones de ésta.

# **Mediante series de Taylor**

La idea básica del tratamiento por series de los sistemas con retardo consiste en convertir la ecuación diferencial en una forma algebraica a través del uso de matrices operacionales de retardo e integración.[1]

La función f0(s) puede expandirse en una serie de Taylor como

$e^{-t\_{m}s}≈1-t\_{m}s+ \frac{t\_{m}^{2}s^{2}}{2!}-\frac{t\_{m}^{3}s^{3}}{3!}+…$2.10

De donde la aproximación mediante una serie de primer orden sería

$e^{-t\_{m}s}≈ f\_{1}\left(s\right)=1-t\_{m}s$2.11

Y mediante una de segundo orden

$e^{-t\_{m}s}≈f\_{2}\left(s\right)=1-t\_{m}s+ \frac{t\_{m}^{2}s^{2}}{2!}$2.12

Utilizaron la aproximación f1(s) en su procedimiento de síntesis de controladores.

También se puede aproximar el tiempo muerto por el cociente de dos series de Taylor de la forma

$e^{-t\_{m}s}≈\frac{e^{-\frac{t\_{m}}{2}s}}{e^{\frac{t\_{m}}{2}s}}-\frac{1-\left(\frac{t\_{m}}{2}\right)s+\frac{(\frac{t\_{m}}{2})^{2}s^{2}}{2!}-\frac{(\frac{t\_{m}}{2})^{3}s^{3}}{3!}+…}{1+\left(\frac{t\_{m}}{2}\right)s+\frac{(\frac{t\_{m}}{2})^{2}s^{2}}{2!}+\frac{(\frac{t\_{m}}{2})^{3}s^{3}}{3!}+…}$2.13

La aproximación de primer orden sería entonces

$e^{-t\_{m}s}≈f\_{2}\left(s\right)= \frac{1-\left(\frac{t\_{m}}{2}\right)s}{1+\left(\frac{t\_{m}}{2}\right)s}=\frac{2-t\_{m}s}{2+t\_{m}s} $2.14

Y la de segundo orden

$e^{-t\_{m}s}≈f\_{4}\left(s\right)= \frac{1-\left(\frac{t\_{m}}{2}\right)s+\left(\frac{t\_{m}^{2}}{8}\right)s}{1+\left(\frac{t\_{m}}{2}\right)s+\left(\frac{t\_{m}^{2}}{8}\right)s}==\frac{8-4t\_{m}s+t\_{m}^{2}s^{2}}{8+4t\_{m}s+t\_{m}^{2}s^{2}}$2.15

# **Aproximaciones de Padé**

Son de las más populares en los estudios de control.[2]

La expresión general de las aproximaciones de Padé es:

$e^{-t\_{m}s}=\frac{1+\sum\_{i=1}^{n}\frac{i!(-t\_{m}s)^{i}}{(2i)!}}{1+\sum\_{i=1}^{n}\frac{i!(t\_{m}s)^{i}}{(2i)!}}$2.16

Para el caso n = 1 (Padé de primer orden) se tiene

$e^{-t\_{m}s}≈\frac{1-(\frac{t\_{m}}{2})s}{1+(\frac{t\_{m}}{2})s}=\frac{2-t\_{m}s}{2+t\_{m}s}$2.17

Que es igual a la aproximación f3(s) anterior y la aproximación de Padé de segundo orden

$e^{-t\_{m}s}≈f\_{5}\left(s\right)=\frac{1-(\frac{t\_{m}}{2})s+(\frac{t\_{m}^{2}}{12})s}{1+(\frac{t\_{m}}{2})s+(\frac{t\_{m}^{2}}{12})s}=\frac{12-6t\_{m}s+t\_{m}^{2}s^{2}}{12+6t\_{m}s+t\_{m}^{2}s^{2}}$2.18

# **Mediante polos y ceros múltiples**

La función de retardo de tiempo se puede definir también como la respuesta de un número infinito de sistemas de primer orden en serie de la forma:

$e^{-t\_{m}s}=\frac{1}{(1+(\frac{t\_{m}}{n})s)^{n}} =\frac{(1-(\frac{t\_{m}}{2n})s)^{n}}{(1+(\frac{t\_{m}}{2n})s)^{n}} $2.19

Entonces, la aproximación de primer orden sería

$e^{-t\_{m}s}≈\frac{1-(\frac{t\_{m}}{2})s}{1+(\frac{t\_{m}}{2})s}=\frac{2-t\_{m}s}{2+t\_{m}s}$2.20

Que resulta ser igual a f3(s) y la de segundo.[2]

$e^{-t\_{m}s}≈f\_{6}\left(s\right)=\frac{(1-(\frac{t\_{m}}{4})s)^{2}}{(1+(\frac{t\_{m}}{4})s)^{2}}=\frac{16-8t\_{m}s+t\_{m}^{2}s^{2}}{16+8t\_{m}s+t\_{m}^{2}s^{2}}$2.21

# **Otras aproximaciones**

Jutan y Rodríguez usaron como parte de su procedimiento de identificación con control P la aproximación

$e^{-t\_{m}s}≈f\_{7}\left(s\right)=\frac{1-0,6143t\_{m}s+0,1247t\_{m}^{2}s^{2}}{1+0,3866t\_{m}s}$2.22

Bogere y Özgen, con el mismo propósito emplearon la expresión

$e^{-t\_{m}s}≈f\_{8}\left(s\right)= 1-0,8647t\_{m}s+0,226t\_{m}^{2}s^{2}$2.23

Marshall

$e^{-t\_{m}s}≈f\_{9}\left(s\right)=\frac{1-0,0625t\_{m}^{2}s^{2}}{1+0,0625t\_{m}^{2}s^{2}}=\frac{16-t\_{m}^{2}s^{2}}{16+t\_{m}^{2}s^{2}}$2.24

Piche (producto)

$e^{-t\_{m}s}≈\frac{1-0,5t\_{m}s+0,125t\_{m}^{2}s^{2}}{1+0,5t\_{m}s+0,125t\_{m}^{2}s^{2}}$2.25

Que es igual a f4(s)

Piche (Laguerre)

$e^{-t\_{m}s}≈\frac{1-0,5t\_{m}s+0,0625t\_{m}^{2}s^{2}}{1+0,5t\_{m}s+0,0625t\_{m}^{2}s^{2}}$2.26

Que es igual a f6(s)

Gradshteyn y Ryzhik

$e^{-t\_{m}s}≈f\_{10}\left(s\right)=\frac{1-0,5t\_{m}s+0,1013t\_{m}^{2}s^{2}}{1+0,5t\_{m}s+0,1013t\_{m}^{2}s^{2}}$2.27

Sthal y Hippe optimizaron la aproximación del tiempo muerto para reproducir la respuesta de frecuencia sobre el ámbito de frecuencias mayor posible y determinaron funciones de transferencia de segundo hasta quinto orden.

Su aproximación de grado dos está dada por la función de transferencia.[2]

$e^{-t\_{m}s}≈f\_{11}\left(s\right)=\frac{1-0,49t\_{m}s+0,0954t\_{m}^{2}s^{2}}{1+0,49t\_{m}s+0,0954t\_{m}^{2}s^{2}}$2.28

# **Aproximación a modelos de orden reducido**

Trabajar con procesos no lineales para el diseño de controladores es dificultoso, debido a ello se procedió a estudiar aproximaciones a modelos lineales de orden reducido logrando obtener un modelo del proceso más sencillo y que presente el mismo comportamiento del proceso no lineal.

Una de las aproximaciones que más se utiliza debido a su simplicidad es la aproximación a un modelo de primer orden con retardo.[20]

Se lo conoce como método empírico y se puede obtener a partir del método de dos puntos de Smith en la curva de reacción del proceso. Se puede aproximar el sistema no lineal a un sistema de modelo de primer orden con retardo, el cual se encuentra representado por la Ecuación.[20]

$\frac{Y\_{(s)}}{u\_{(s)}}=\frac{Ke^{-t\_{0}s}}{τs+1}$2.29

Dónde:

Y(s): Transformada de Laplace de la variable controlada.

u(s): Transformada de Laplace de la variable manipulada.

k: Ganancia en estado estacionario del sistema.

T0: Retardo de tiempo del sistema.

τ: Constante de tiempo del sistema.

# **Identificación del comportamiento dinámico de una planta con retardo de tiempo**

En las últimas décadas, los sistemas dinámicos retardados han atraído la atención de investigadores de diversos campos, como las matemáticas, la biología, la economía, la física, la ingeniería y muchos otros como fines industriales. Muchos sistemas naturales se modelan matemáticamente mediante ecuaciones diferenciales no lineales que contienen uno o más retardos temporales. Algunos ejemplos exitosos son la producción de sangre en pacientes con leucemia, la dinámica de los sistemas ópticos, la dinámica de la población, el modelo fisiológico, la fuerza de Lorentz con potenciales de Liénard-Weichert, red neuronal con tres neuronas, control de retroalimentación de retardo y sincronización, etc. La presencia de un retardo en un sistema hace que el sistema sea de dimensiones infinitas, y puede conducir a una respuesta inestable y oscilatoria. En particular, el retardo temporal de un sistema no lineal puede dar lugar a varios fenómenos complejos como la bifurcación, el caos, el hipercaos, la multiestabilidad, etc. [36]

Las arquitecturas cognitivas modernas, como ACT-R, permiten a los investigadores construir modelos computacionales de comportamiento que reflejan adecuadamente la complejidad de la cognición humana, al tiempo que formalizados. Las arquitecturas cognitivas suelen basarse en estudios empíricos sobre el comportamiento y en investigaciones neurofisiológicas. Utilizando un modelo cognitivo de toma de decisiones, es posible responder a preguntas como "¿cómo se comporta un tomador de decisiones típico en una situación particular? o "qué se puede esperar, en el mejor o peor de los casos, de un decisor".

Los modelos cognitivos suelen centrarse en fenómenos cognitivos específicos, mientras que las arquitecturas cognitivas se ocupan de la estructura general del sistema cognitivo en diferentes tareas.[33]

Los modelos computacionales de la cognición proporcionan una interfaz para conectar herramientas y métodos matemáticos avanzados. En distintos artículos se observa cómo un modelo computacional de aprendizaje basado en instancias es usado, implementado en la arquitectura cognitiva proponen un enfoque para obtener reformulaciones matemáticas de tales modelos cognitivos que mejoren su trazabilidad computacional.

Para la tarea de toma de decisiones dinámicas realizamos un estudio de simulación para analizar los parámetros centrales del modelo. Mostramos cómo las técnicas de optimización matemática pueden aplicarse para identificar eficientemente los valores óptimos de los parámetros con respecto a diferentes objetivos de optimización. Más allá de estas contribuciones metodológicas, nuestro análisis revela la sensibilidad de esta tarea en particular con respecto a los ajustes iniciales y proporciona nuevos conocimientos sobre los valores óptimos de los parámetros.[33]

Se introduce la retroalimentación con retardo de tiempo para controlar de los comportamientos dinámicos del sistema y se presentan algunas simulaciones numéricas para validar el efecto del control. Estas proporcionan una base teórica para el diseño y el control del sistema.[15]

# **El predictor de Smith como estrategia de control de plantas con retardo de tiempo de El predictor de Smith (PS)**

Durante los últimos 25 años han existido diversos cambios o modificaciones del predictor Smith a fin de: mejorar la regulación del predictor Smith para tipos de perturbaciones medibles, ser usadas en plantas inestables, sistema más robusto y mejoría en la puesta a punto para aplicaciones industriales.[22]

En todo punto [22] nos indica un problema extra importante el cual está relacionado a la parametrización, poder encontrar todos los controladores estabilizadores para una planta, algunos autores como Yamada en “A design method for Smith predictor for minimum-phase time-delay plants” optó por parametrizar todos los predictores Smith modificados para plantas cuya fase era mínima respecto a su retardo de tiempo, otro como Majhi en “Modified Smith predictor and controller for processes with time delay“ a fin de solucionar lo indicado inicialmente propusieron un nuevo predictor con 3 controladores, mientras que Matausek en “On the Modified Smith Predictor for Controlling a Process with an Integrator and Long Dead-Time” desarrolló un esquema alternativo a dos grados de libertad el cual poseía un nuevo bucle de retroalimentación; sin embargo, poseía un mal rendimiento para un proceso incierto con acción integral. Podemos afirmar, mediante los casos mencionados anteriormente, lo propuesto inicialmente que indicaba las diversas nuevas opciones y nuevos cambios o modificaciones del predictor Smith.

Según [23] nos indica que otros tipos de controladores como los PID pueden ser diseñados a fin de aproximar a una solución a fin de controlar los tiempos muertos. Por ellos si no es un tipo de PID con retroalimentación arrojará pequeñas diferencias del sistema.

Los circuitos de retroalimentación en un sistema de control siempre se asociarán con: tiempos muertos debido al fino tiempo de detección, señal de un proceso, computación del control de entrada y acción [21]. Ello nos indicará que para los controles y en este caso, para tanques de mezclado continúo, existirán tiempos muertos o demoras en las que un sistema podrá indicar una actividad iniciada, por ejemplo, el aumento de temperatura de un tanque.

Una de las formas que tenemos de compensar los efectos de una llegada de retroalimentación tardía es mediante la aplicación de predictores de esta misma retroalimentación, un ejemplo claro de ello son los predictores Smith [37]. Con lo anterior indicamos el gran beneficio que nos trae el uso de predictores Smith puesto que podemos cubrir esos “retrasos” existentes en el control y corregir más rápido fluctuaciones de temperatura en los tanques de mezclado continuo.

Otro ejemplo donde se puede observar el manejo de tiempo muerto del predictor de Smith es en un modelo de sistema climático dinámico de un invernadero [13], ya que los factores que intervienen en este interactúan uno con otro. El predictor ayuda la optimización del control de estos factores basados en el balance total de energía-masa dentro del invernadero y descritos en dos ecuaciones diferenciales no lineales, una de la latencia y sensibilidad del calor y otra del balance del vapor del agua.[13]![Diagrama  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.18.** Sistema dinámico de un invernadero

Fuente: Referencia [13]

Dado que el predictor Smith tiene la capacidad del control de esa cantidad de variables, se puede asegurar que para un tanque de mezclado continuo podrá aplicarse con mayor eficiencia, ya que las variables de control en esta situación son muchas más accesibles y fáciles de predecir con este algoritmo.

Existen diferentes enfoques que pueden emplear soluciones alternas [16], que requieren métodos, conceptos y procedimientos matemáticos, como el PID tradicional, PID de dos grados de libertad o los compensadores de tiempo muerto (DTC), los cuales muestran una gran gama de enfoques de control para estos casos ya sean simples o complejos.

# **Diseño del controlador**

El diseño del sistema de control del proceso se desarrolló sobre la base del siguiente modelo:[22]

$G\_{0}\left(s\right)=\frac{Y(s)}{U(s)}=\frac{1.48}{(13.4s+1)(2s+1)}e^{-105.5s}$ 2.30

Inicialmente se diseñó un sistema de control con controlador PI, de forma similar a como se realiza actualmente el control de dicho proceso. Seguidamente se diseñó un segundo sistema de control de este proceso con estructura clásica del predictor de Smith y con el mismo ajuste del controlador PI. Luego se desarrollaron trabajos de simulación de ambos sistemas de control considerando la ocurrencia de una perturbación a un determinado tiempo.[5]

![Gráfico, Gráfico de líneas  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.19.** Sistema de control PI+PS

Fuente: Referencia [13]

De la figura mostrada, se observa que ambos sistemas alcanzan la referencia de 105°C, sólo que el sistema con predictor de Smith se demora 213 s, mientras que el sistema con controlador PI poco más del triple de tiempo. Asimismo, el sistema de control con PI se demora 554 s en rechazar el efecto de la perturbación externa, mientras que el sistema con predictor de Smith lo hace en menos tiempo.[5]

A continuación, se elaboró el sistema de control del proceso objeto de estudio con estructura modificada del predictor de Smith con controlador PI y con el mismo ajuste que en los sistemas de control anteriores.[5]

Los resultados de la simulación de los tres sistemas diseñados considerando la ocurrencia de una perturbación al mismo tiempo planteado en la simulación anterior, se muestran en la siguiente figura:

![Gráfico  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.20.** Sistema de control PI+PS+Mod Inv.

Fuente: Referencia [5]

Se observa que, ante un paso en la referencia ambos sistemas de control con predictor de Smith se comportan de forma similar, sin embargo, el tiempo que demora el sistema con predictor de Smith y estructura modificada en rechazar el efecto de la perturbación externa es de 215 s, mientras que el sistema con predictor de Smith y estructura clásica es de 315 s. [5]

De esta manera, el sistema de control con estructura modificada del predictor de Smith rechaza el efecto de las perturbaciones externas no medibles 100 s más rápido que el sistema de control con estructura clásica del predictor de Smith.

![Gráfico, Gráfico de líneas  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.21.** Sistema de control modificado PI, PI+PS y PI+PS+Mod Inv.

Fuente: Referencia [5]

En este último gráfico se presenta una ampliación de los resultados que se mostraron anteriormente, en la cual se observa la efectividad en el rechazo de las perturbaciones del sistema de control con estructura modificada del predictor de Smith en comparación con el sistema de control con estructura clásica del predictor de Smith.

En la siguiente figura se muestran los resultados de la simulación de las señales de error que se causan como resultado del efecto de las perturbaciones externas no medibles en el sistema de control del proceso objeto de estudio con estructura clásica del predictor de Smith y en el sistema de control del mismo proceso con estructura modificada del predictor de Smith.[5]

![Interfaz de usuario gráfica, Gráfico, Gráfico de líneas  Descripción generada automáticamente](data:image/png;base64...)

**Figura 2.22.** Resultados de la simulación de las señales de erro en los sistemas de control con estructura clásica y estructura modificada del Predictor Smith

Fuente: Referencia [5]

A partir de esto, se observa que, en el sistema de control modificado, la señal de error que se origina como resultado de las perturbaciones externas no medibles se elimina más rápido que en el sistema de control con estructura clásica del predictor de Smith. Estos resultados demuestran la efectividad en el rechazo de las perturbaciones externas no medibles de la estructura propuesta del predictor de Smith.[5]

De esta forma se ha demostrado mediante las simulaciones presentadas que el diseño del predictor de Smith cumple con su finalidad, siendo comparada con un modelo modificado de este mismo, siendo en ambos casos un resultado similar pero efectivo.

CAPÍTULO III

**DESARROLLO DEL TRABAJO DE LA TESIS**

En el presente capítulo se desarrolla el objetivo de la tesis, que consiste en diseñar e implementar un módulo de control de temperatura para un tanque de mezclado continúo usando un predictor Smith.

# **Descripción del método**

La metodología se divide en 5 etapas, que está directamente asociado a los objetivos específicos. Los mostrado se da en la figura 3.1.

![](data:image/png;base64...)

**Figura. 3.1** Metodología de diagnóstico del proyecto.

# **Modelamiento matemático**

## **Diseño e implementación del prototipo**

El mezclador de tanques de dosificación utilizado en este proyecto se muestra en la figura 3.2. Es de la serie DVTX.

![](data:image/png;base64...)

**Figura. 3.2** Mezclador o agitador para tanques de dosificación.

El sensor de temperatura seleccionado es el DS18B20 mostrado en la figura 3.3.

![](data:image/png;base64...)

**Fig. 3.3** Sensor de temperatura DS18B20.

La válvula de flujo seleccionada es de fabricación propia. Se muestra en la figura 3.4.

![](data:image/png;base64...)

**Figura. 3.4** Válvula de flujo de fabricación propia.

## **Descripción del prototipo físico**

El sistema físico es un tanque de agitación constante con múltiples perturbaciones (O. Camacho, C. Smith). Dicho sistema se muestra en la figura 3.5.

![](data:image/png;base64...)

**Figura 3.5** Esquema del sistema físico.

El tanque de mezcla que se muestra en la figura 3.2. el tanque recibe dos corrientes, una corriente caliente, $W\_{1}\_{\left(t\right)}$, y una corriente fría, $W\_{2}\_{\left(t\right)}$. La temperatura de salida se mide en un punto a $125 ft$ aguas abajo del tanque. Se aceptan las siguientes suposiciones:

* El volumen de líquido en el tanque se considera constante.
* El contenido del tanque está bien mezclado.
* El tanque y la tubería están bien aislados.

El transmisor de temperatura está calibrado para un rango de $100 °F$ a $200 °F$. La Tabla 3.1 muestra las condiciones de estado estable y otra información de funcionamiento.

**Tabla 3.1**. Parámetros que intervienen en el sistema.

|  |  |  |
| --- | --- | --- |
| **Variable** | **Descripción** | **Valor** |
| $$W\_{1}$$ | Flujo másico del fluido caliente | $$250 \frac{lb}{min}$$ |
| $$W\_{2}$$ | Flujo másico del fluido frío | $$191.17 \frac{lb}{min}$$ |
| $$C\_{P\_{1}}$$ | Capacidad calorífica a presión constante del fluido caliente | $$0.8 \frac{BTU}{lb×°F}$$ |
| $$C\_{P\_{2}}$$ | Capacidad calorífica a presión constante del fluido frío | $$1.0 \frac{BTU}{lb×°F}$$ |
| $$C\_{P\_{3}}$$ | Capacidad calorífica a presión constante del fluido dentro del tanque | $$0.9 \frac{BTU}{lb×°F}$$ |
| $$C\_{V\_{3}}$$ | Capacidad calorífica a volumen constante del fluido dentro del tanque | $$0.9 \frac{BTU}{lb×°F}$$ |
| $$T\_{1}$$ | Temperatura del fluido caliente | $$250 °F$$ |
| $$T\_{2}$$ | Temperatura del fluido frío | $$50 °F$$ |
| $$T\_{SP}$$ | Temperatura de referencia | $$150 °F$$ |
| $$T\_{3}$$ | Temperatura del fluido dentro del tanque | $$150 °F$$ |
| $$ρ$$ | Densidad del fluido dentro del tanque | $$62.4 \frac{lb}{ft^{3}}$$ |
| $$V$$ | Volumen del fluido dentro del tanque | $$15 ft^{3}$$ |
| $$V\_{P}$$ | Posición del obturador de la válvula. Valores entre 0 (válvula cerrada) y 1 (válvula abierta) | $$0.478$$ |
| $$\overline{TO}$$ | Señal de salida del transmisor de temperatura en una escala de 0 a 1 | $$0.5$$ |
| $$C\_{VL}$$ | Coeficiente de flujo de la válvula | $$12 \frac{gpm}{psi^{\frac{1}{2}}}$$ |
| $$∆P\_{V}$$ | Caída de presión en la válvula | $$16 psi$$ |
| $$τ\_{T}$$ | Constante de tiempo del sensor de temperatura | $$0.5 min$$ |
| $$τ\_{V\_{p}}$$ | Constante de tiempo del actuador | $$0.4 min$$ |
| $$A$$ | Diámetro de la tubería | $$0.2006 ft^{2}$$ |
| $$L$$ | Longitud de la tubería | $$125 ft$$ |
| $$\overline{m}$$ | Fracción de la salida del controlador en una escala de 0 a 1 | $$0.478 CO$$ |

Las siguientes ecuaciones constituyen el modelo de proceso:

1. Balance de energía alrededor del tanque de agitación:

$$W\_{1}\_{\left(t\right)}C\_{P\_{1}}\_{\left(t\right)}T\_{1}\_{\left(t\right)}+W\_{2}\_{\left(t\right)}C\_{P\_{2}}\_{\left(t\right)}T\_{2}\_{\left(t\right)}-\left(W\_{1}\_{\left(t\right)}+W\_{2}\_{\left(t\right)}\right)C\_{P\_{3}}\_{\left(t\right)}T\_{3}\_{\left(t\right)}=VρC\_{V\_{3}}\_{\left(t\right)}\frac{dT\_{3}}{dt}…\left(3.1\right)$$

1. Retraso de señal entre el tanque y el sensor:

$$T\_{4}\_{\left(t\right)}=T\_{3}\_{\left(t-t\_{0}\right)}…\left(3.2\right)$$

1. Tiempo muerto:

$$t\_{0}=\frac{LAρ}{W\_{1}\_{\left(t\right)}+W\_{2}\_{\left(t\right)}}…\left(3.3\right)$$

1. Señal del transmisor:

$$\frac{dTO\_{\left(t\right)}}{dt}=\frac{1}{τ\_{T}}\left[\frac{T\_{4}\_{(t)}-100}{100}-TO\_{\left(t\right)}\right]…\left(3.4\right)$$

1. Posición del obturador de la válvula:

$$\frac{dV\_{p}\_{\left(t\right)}}{dt}=\frac{1}{τ\_{V\_{p}}}\left[m\_{\left(t\right)}-V\_{p}\_{\left(t\right)}\right]…\left(3.5\right)$$

1. Ecuación de la válvula:

$$W\_{2}\_{\left(t\right)}=\frac{500}{60}C\_{VL}V\_{P}\_{\left(t\right)}\sqrt{G\_{f}∆P\_{V}}…\left(3.6\right)$$

Se utilizan las ecuaciones presentadas para obtener los parámetros del modelo FOPDT (first-order-plus dead time): $K = -0.983 fraction TO / fraction CO$, $τ=2.369 min$ y $t\_{0}=4.074 min$. Entonces:

$$G\_{(s)}=\frac{K}{τs+1}e^{-t\_{0}s}...(3.7)$$

$$G\_{(s)}=\frac{-0.983}{(2.369 min)s+1}e^{-(4.074 min)s}...(3.8)$$

## **Metodología propuesta para la obtención del modelo matemático.**

Para obtener el modelo se usa el método **FIT por Smith C. and Corripio A** que consiste en variar el flujo de agua fría de su estado estable a un 5% por encima de este.

## **Validación del modelo matemático.**

Luego de la obtención del modelo matemático y la construcción del sistema donde el diagrama se presenta en la figura 3.3 se realiza la comparación a tiempo real de las respuestas de ambos modelos. El resultado del modelo se muestra en la figura 3.6.

![](data:image/png;base64...)

**Figura. 3.6.** Respuesta del modelo a una señal de entrada tipo escalón y un disturbio tipo escalón

## **Diseño de controlador**

### **Diseño del controlador PI**

A continuación, se presenta el diagrama de bloques del sistema a lazo cerrado usando un controlador PI en la figura 3.7.

**![](data:image/png;base64...)**

**Figura. 3.7.** Diagrama de bloques del sistema a lazo cerrado usando un controlador PI.

Si la función de transferencia es:

$$F\_{\left(s\right)}=\frac{K\_{p}}{Ts+1}e^{-Ls}...(3.9)$$

Y el controlador es:

$$C\_{\left(s\right)}=K\_{c}\left(1+\frac{1}{T\_{i}s}\right)...(3.10)$$

La función de transferencia a lazo cerrado es:

$$F\_{lc}\_{\left(s\right)}=\frac{C\_{\left(s\right)}F\_{\left(s\right)}}{1+C\_{\left(s\right)}F\_{\left(s\right)}}...(3.11)$$

$$F\_{lc}\_{\left(s\right)}=\frac{\left[K\_{c}\left(1+\frac{1}{T\_{i}s}\right)\right]\left[\frac{K\_{p}}{Ts+1}e^{-Ls}\right]}{1+\left[K\_{c}\left(1+\frac{1}{T\_{i}s}\right)\right]\left[\frac{K\_{p}}{Ts+1}e^{-Ls}\right]}$$

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}\frac{T\_{i}s+1}{T\_{i}s\left(Ts+1\right)}e^{-Ls}}{1+K\_{c}K\_{p}\frac{T\_{i}s+1}{T\_{i}s\left(Ts+1\right)}e^{-Ls}}$$

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}\left(T\_{i}s+1\right)e^{-Ls}}{T\_{i}s\left(Ts+1\right)+K\_{c}K\_{p}\left(T\_{i}s+1\right)e^{-Ls}}...(3.12)$$

Sea $T=T\_{i}$:

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}e^{-Ls}}{T\_{i}s+K\_{c}K\_{p}e^{-Ls}}$$

Sea $e^{-Ls}=1-Ls$:

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}e^{-Ls}}{T\_{i}s+K\_{c}K\_{p}(1-Ls)}$$

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}e^{-Ls}}{(T\_{i}-K\_{c}K\_{p}L)s+K\_{c}K\_{p}}$$

$$F\_{lc}\_{\left(s\right)}=\frac{e^{-Ls}}{(T\_{i}-K\_{c}K\_{p}L/K\_{c}K\_{p}) s+1}$$

Sea $T\_{c}=(T\_{i}-K\_{c}K\_{p}L)/K\_{c}K\_{p}$ de donde se deduce que:

$$K\_{c}=\frac{T}{(T\_{c}+L)K\_{p}}$$

### **Diseño del controlador PID**

A continuación, se presenta el diagrama de bloques del sistema a lazo cerrado usando un controlador PID en la figura 3.8.

**![](data:image/png;base64...)**

**Figura. 3.8.** Diagrama de bloques del sistema a lazo cerrado usando un controlador PID.

Si la función de transferencia es:

$$F\_{\left(s\right)}=\frac{K\_{p}}{Ts+1}e^{-Ls}...(3.9)$$

Y el controlador es:

$$C\_{\left(s\right)}=K\_{c}\left(1+\frac{1}{T\_{i}s}\right)\frac{(1+T\_{d}s)}{( 1+αT\_{d}s)}...(3.10)$$

La función de transferencia a lazo cerrado es:

$$F\_{lc}\_{\left(s\right)}=\frac{C\_{\left(s\right)}F\_{\left(s\right)}}{1+C\_{\left(s\right)}F\_{\left(s\right)}}...(3.11)$$

$$F\_{lc}\_{\left(s\right)}=\frac{\left[K\_{c}\left(1+\frac{1}{T\_{i}s}\right)\frac{(1+T\_{d}s)}{( 1+αT\_{d}s)}\right]\left[\frac{K\_{p}}{Ts+1}e^{-Ls}\right]}{1+\left[K\_{c}\left(1+\frac{1}{T\_{i}s}\right)\frac{(1+T\_{d}s)}{( 1+αT\_{d}s)}\right]\left[\frac{K\_{p}}{Ts+1}e^{-Ls}\right]}$$

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}\frac{T\_{i}s+1}{T\_{i}s\left(Ts+1\right)}\frac{(1+T\_{d}s)}{( 1+αT\_{d}s)}e^{-Ls}}{1+K\_{c}K\_{p}\frac{T\_{i}s+1}{T\_{i}s\left(Ts+1\right)}\frac{(1+T\_{d}s)}{( 1+αT\_{d}s)}e^{-Ls}}$$

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}\left(T\_{i}s+1\right)(1+T\_{d}s)e^{-Ls}}{T\_{i}s\left(Ts+1\right)( 1+αT\_{d}s)+K\_{c}K\_{p}\left(T\_{i}s+1\right)(1+T\_{d}s)e^{-Ls}}...(3.12)$$

Sea $T=T\_{i}$:

$$F\_{lc}\_{\left(s\right)}=\frac{(1+T\_{d}s)K\_{c}K\_{p}e^{-Ls}}{Ts( 1+αT\_{d}s)+(1+T\_{d}s)K\_{c}K\_{p}e^{-Ls}}...(3.13)$$

Sea $e^{-Ls}=\frac{1-L/2s}{1+L/2s}$

$$F\_{lc}\_{\left(s\right)}=\frac{(1+\frac{L}{2}s)K\_{c}K\_{p}(1+T\_{d}s)e^{-Ls}}{Ts(1+\frac{L}{2}s)( 1+αT\_{d}s)+K\_{c}K\_{p}(1-\frac{L}{2}s)(1+T\_{d}s)}...(3.14)$$

Sea $\frac{L}{2}=T\_{d}$

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}(1+\frac{L}{2}s)e^{-Ls} }{Ts( 1+α\frac{L}{2}s)+K\_{c}K\_{p}(1-\frac{L}{2}s)}...(3.15)$$

Sea $K\_{c}=\frac{T}{K\_{p}L/(1-α)}$

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}^{}e^{-Ls}}{\frac{2}{L}(\frac{TLα}{2}s+T(1-α)-\frac{K\_{c}K\_{p}L}{2})}$$

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}^{}e^{-Ls}}{\frac{2}{L}(\frac{TLα}{2}s+K\_{c}K\_{p}L-\frac{K\_{c}K\_{p}L}{2})}$$

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}^{}e^{-Ls}}{(Tαs+K\_{c}K\_{p})}$$

$$F\_{lc}\_{\left(s\right)}=\frac{e^{-Ls}}{(\frac{Tα}{K\_{c}K\_{p}}s+1)}...(3.16)$$

Sea $T\_{c}=\frac{Tα}{K\_{c}K\_{p}}$ y teniendo en cuenta $K\_{c}=\frac{T}{K\_{p}L/(1-α)}$ se deduce:

$$\frac{Tα}{T\_{c}}=\frac{T(1-α)}{L}$$

$$α=\frac{T\_{c}}{L+T\_{c}}...(3.17)$$

Reemplazando en $T\_{c}=\frac{Tα}{K\_{c}K\_{p}}$ se obtiene:

$$K\_{c}=\frac{T}{K\_{p}(L+T\_{c})}...(3.18)$$

### **Diseño de un controlador predictor Smith**

A continuación, se presenta el diagrama de bloques del sistema a lazo cerrado usando un controlador predictor Smith. El diagrama de bloques se muestra en la figura 3.9.

Para diseñar el controlador se optó por el método de rediseño, así es que primero realizamos el diagrama de bloques donde la parte enmarcada es la parte digital

**![](data:image/png;base64...)**

**Figura. 3.9.** Diagrama de bloques del sistema a lazo cerrado usando un controlador predictor Smith.

Reduciendo el diagrama se obtiene la figura 3.10.

**![](data:image/png;base64...)**

**Fig. 3.10.** Diagrama de bloques del sistema a lazo cerrado usando un controlador predictor Smith.

Si asumimos que $P\_{m}(s)=P(s)$, se tendría que diseñar al controlador siendo la planta $G\_{m}(s)$. Si la función de transferencia es:

$$F\_{\left(s\right)}=\frac{K\_{p}}{Ts+1}...(3.19)$$

Y el controlador es:

$$C\_{\left(s\right)}=K\_{c}\left(1+\frac{1}{T\_{i}s}\right)...(3.20)$$

La función de transferencia a lazo cerrado es:

$$F\_{lc}\_{\left(s\right)}=\frac{C\_{\left(s\right)}F\_{\left(s\right)}}{1+C\_{\left(s\right)}F\_{\left(s\right)}}...(3.21)$$

$$F\_{lc}\_{\left(s\right)}=\frac{\left[K\_{c}\left(1+\frac{1}{T\_{i}s}\right)\right]\left[\frac{K\_{p}}{Ts+1}\right]}{1+\left[K\_{c}\left(1+\frac{1}{T\_{i}s}\right)\right]\left[\frac{K\_{p}}{Ts+1}\right]}$$

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}\frac{T\_{i}s+1}{T\_{i}s\left(Ts+1\right)}}{1+K\_{c}K\_{p}\frac{T\_{i}s+1}{T\_{i}s\left(Ts+1\right)}}$$

$$F\_{lc}\_{\left(s\right)}=\frac{K\_{c}K\_{p}\left(T\_{i}s+1\right)}{T\_{i}s\left(Ts+1\right)+K\_{c}K\_{p}\left(T\_{i}s+1\right)}...(3.22)$$

Sea $T=T\_{i}$:

$$F\_{lc}\_{\left(s\right)}=\frac{1}{T/K\_{c}K\_{p}s+1}...(3.23)$$

Sea $T\_{c}=T\_{i}/K\_{c}K\_{p}$ de donde se deduce que:

$$K\_{c}=\frac{T}{T\_{c}K\_{p}}...(3.24)$$

### **Diseño de los controladores mediante Matlab**

A continuación se presenta el script de Matlab donde se diseñará el controlador a partir de las ecuaciones anteriores.

%Parámetros de la planta

k=-0.983

tau=2.369

L=4.074

s = tf('s');

%FILTRO

F = s/s

F.InputName = 'dy' ;

F.OutputName = 'dp' ;

%process

P = exp(-L\*s) \* k/(tau\*s+1);

P.InputName = 'u';

P.OutputName = 'y0';

%Prediction model

Gp = k/(tau\*s+1);

Gp.InputName = 'u';

Gp.OutputName = 'yp';

%Dp = exp(-L\*s) ;

Dp = 1-L\*s

Dp.InputName = 'yp'; Dp.OutputName = 'y1';

%CONTROLADOR

tc= (1/8)\*L

ti=tau

kc=tau/(k\*(L+tc))

CPI=kc\*(1+1/(ti\*s))

CPI.InputName = 'e';

CPI.OutputName = 'u';

td=L/2

kc=tau/(k\*(L+tc))

alfa=tc/(L+tc)

CPID=kc\*(1+1/(ti\*s))\*((td\*s+1)/(alfa\*td\*s+1))

CPID.InputName = 'e';

CPID.OutputName = 'u';

ti=tau

kc=tau/(k\*tc)

CPI2=kc\*(1+1/(ti\*s))

CPI2.InputName = 'e';

CPI2.OutputName = 'u';

% Assemble closed-loop model from [y\_sp,d] to y

Sum0 = sumblk('e = ysp - y0');

Sum1 = sumblk('e = ysp - yp - dp');

Sum2 = sumblk('y = y0 + d');

Sum3 = sumblk('dy = y - y1');

T1 = connect(P,CPI,Sum0,Sum2,{'ysp','d'},'y');

T2 = connect(P,CPID,Sum0,Sum2,{'ysp','d'},'y');

T = connect(P,Gp,Dp,CPI2,F,Sum1,Sum2,Sum3,{'ysp','d'},'y');

step(T1,'b',T2,'r-',T,'g--')

grid on

legend('PI Controller','pid controller','predictor smith'

## **Diagrama de bloques estructurales**

El sistema tanque de agitación se representa mediante el diagrama de bloques mostrado en la figura 3.11:

**![](data:image/png;base64...)**

**Fig. 3.11.** Diagrama de bloques del sistema a lazo cerrado usando un controlador predictor Smith.

El sistema se puede simplificar tal como se muestra en la figura 3.12:

Temperatura de referencia R(s)

Perturbación d(s)

Temperatura de salida y(s)

**Fig. 3.12.** Simplificación del sistema.

## **Diagrama de flujos funcional y explicación de cada una de las etapas y su interrelación**

El diagrama de flujo funcional se muestra en la figura 3.13.

![](data:image/png;base64...)

**Fig. 3.13.** Diagrama de flujo funcional

## **Pseudocódigo del programa de simulación y descripción de cada etapa de la secuencia del programa.**

clc

close

clear

%% Planta

k=-0.983;

tau=2.369;

L=4.074;

P=tf([0 k],[tau 1],'IODelay',L);

%% Controlador PI

s=tf('s');

C\_PI=(-0.9399\*s-0.3968)/(2.369\*s);

%% Controlador PID

s=tf('s');

C\_PID=(-1.915\*s^2 - 1.748\*s - 0.3968)/(2.369\*s);

%% Tiempo de muestreo

Plc\_PI=feedback(series(C\_PI,P),1);

Plc\_PID=feedback(series(C\_PID,P),1);

figure(1),hold on,step(Plc\_PI),step(Plc\_PID);

Ts=input('Tiempo de muestreo: ');

%% Discretización

Pz=c2d(P,Ts,'zoh');

Cz\_PI=c2d(C\_PI,Ts,'tustin');

Cz\_PID=c2d(C\_PID,Ts,'matched');

Plcz\_PI=feedback(series(Cz\_PI,Pz),1);

Plcz\_PID=feedback(series(Cz\_PID,Pz),1);

figure(2),hold on,step(Plcz\_PI),step(Plcz\_PID);

## **Diseño de los experimentos de validación de resultados (simulación o implementación) Condiciones iniciales, características y definición de parámetros del experimento.**

En el experimento se analizará las características de la variable de salida del sistema de control ante diversas perturbaciones a la salida de la planta.. Las características de la señal de la variable de salida son descritas en la norma IEEE 1031 , se definen como:

* Tiempo de respuesta (Response time). Duración desde el cambio de paso de la señal de salida hasta que alcance el 90% del valor en estado estacionario, antes de algún sobre impulso.
* Tiempo de establecimiento (Settling time). Duración desde el cambio de paso de la señal de salida hasta el tiempo de salida estable dentro del +/-5% de la salida de control requerida.
* Sobre impulso (Overshoot). Sobre impulso sobre el valor de estado estacionario. Este valor debe ser menor al 20% para ser considerado estable.

**![](data:image/png;base64...)**

**Figura 3.14.** Definición de tiempos de respuesta y estabilización

Los escenarios de análisis del desempeño del controlador son los siguientes:

* Control automático frente a perturbaciones a la salida del tipo escalón
* Control automático frente a entradas tipo escalón

Los criterios de aceptación son mostrados en la tabla 3.2.

**Tabla 3.2.** Criterios de aceptación de respuesta de tensión

|  |  |
| --- | --- |
| Criterio | Valores de aceptación |
| Tiempo de respuesta | < 5 min |
| Tiempo de estabilización | < 15 min |
| Sobrepaso | < 20% |

**Tabla 3.3.** Descripción de casos del experimento

|  |  |  |
| --- | --- | --- |
| Caso | Condiciones | Descripción |
| Caso 1 | Modo de control | PI,PID,SMITH |
| Evento | Paso de temperatura de referencia 5% |
| Parámetros a evaluar | Tiempo de establecimiento, Sobreimpulso |
| Caso 2 | Modo de control | PI,PID,SMITH |
| Evento | Disturbio |
| Parámetros a evaluar | Tiempo de establecimiento, Sobreimpulso |

CAPÍTULO IV

**RESULTADOS Y ANÁLISIS**

# **4.1. Resultados de los experimentos.**

Se muestran las respuestas del sistema de control en la figura 4.1 al sintonizar con un $T\_{C}=2L$ donde el controlador es controlador PI (trazo azul continuo), controlador PID (trazo rojo continuo) y predictor Smith (trazo verde discontinuo).

**![](data:image/png;base64...)**

**Figura. 4.1.** Respuestas del sistema de control de los tres diferentes tipos de controladores sintonizados con un $T\_{C}=2L$.

Se muestran las respuestas del sistema de control en la figura 4.2 al sintonizar con un $T\_{C}=L$ donde el controlador es controlador PI (trazo azul continuo), controlador PID (trazo rojo continuo) y predictor Smith (trazo verde discontinuo).

* ![](data:image/png;base64...)

**Fig. 4.2.** Respuestas del sistema de control de los tres diferentes tipos de controladores sintonizados con un $T\_{C}=L$

Se muestran las respuestas del sistema de control en la figura 4.3 al sintonizar con un $T\_{C}=L/2$ donde el controlador es controlador PI (trazo azul continuo), controlador PID (trazo rojo continuo) y predictor Smith (trazo verde discontinuo).

![](data:image/png;base64...)

**Fig. 4.3.** Respuestas del sistema de control de los tres diferentes tipos de controladores sintonizados con un $T\_{C}=L/2$.

Se muestran las respuestas del sistema de control en la figura 4.4 al sintonizar con un $T\_{C}=L/4 $donde el controlador es controlador PI (trazo azul continuo), controlador PID (trazo rojo continuo) y predictor Smith (trazo verde discontinuo).

![](data:image/png;base64...)

**Fig. 4.4.** Respuestas del sistema de control de los tres diferentes tipos de controladores sintonizados con un $T\_{C}=L/4$.

# **4.2. Tablas resúmenes de los resultados.**

Se muestra en la tabla 4.1 los parámetros de performance como el sobreimpulso o $M\_{P}$ y el tiempo de establecimiento o $t\_{s}$ al 5%.

**Tabla 4.1.** Parámetros de performance para el análisis de las respuestas de salida

|  |  |  |  |  |  |  |  |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
|  | $$T\_{C}=2L$$ | | $$T\_{C}=L$$ | | $$T\_{C}=L/2$$ | | $$T\_{C}=L/4$$ | |
| $$M\_{P}$$ | $$t\_{s}$$ | $$M\_{P}$$ | $$t\_{s}$$ | $$M\_{P}$$ | $$t\_{s}$$ | $$M\_{P}$$ | $$t\_{s}$$ |
| Controlador PI | - | $$26.1 min$$ | $$4.03\%$$ | $$13.7 min$$ | $$17.6\%$$ | $$20.5 min$$ | $$30.2\%$$ | $$27.6 min$$ |
| Controlador PID | - | $$28.6 min$$ | - | $$15.8 min$$ | $$4.48\%$$ | $$8.67 min$$ | $$15.1\%$$ | $$17.2 min$$ |
| Controlador Smith | - | $$28.6 min$$ |  | $$15.8 min$$ | $$4.48\%$$ | $$8.67 min$$ | $$15.1\%$$ | $$17.2 min$$ |

# **4.3. Gráficos de los resultados.**

Los parámetros mostrados en la tabla 4.1 se muestran en la figura 4.2. Solo se destaca la presencia del tiempo de asentamiento al 5% y hay ausencia de un sobreimpulso para la aplicación de los tres controladores sintonizados con un $T\_{C}=2L$.

![](data:image/png;base64...)

**Figura 4.5.** Exposición de los parámetros de performance en las respuestas de los sistemas de control con diferentes controladores sintonizados con un $T\_{C}=2L$.

![](data:image/png;base64...)

**Figura 4.6.** Exposición de los parámetros de performance en las respuestas de los sistemas de control con diferentes controladores sintonizados con un $T\_{C}=L$.

![](data:image/png;base64...)

**Figura 4.7.** Exposición de los parámetros de performance en las respuestas de los sistemas de control con diferentes controladores sintonizados con un $T\_{C}=L/2$.

![](data:image/png;base64...)

**Figura 4.8.** Exposición de los parámetros de performance en las respuestas de los sistemas de control con diferentes controladores sintonizados con un $T\_{C}=L/4$.

# **4.4. Comparaciones con los resultados de otras técnicas.**

De acuerdo a la tabla 4.1, el efecto del controlador PID y Predictor Smith da un mismo tiempo de establecimiento de 28.6 minutos y el controlador PI da un tiempo de 26.1 minutos. El sobreimpulso está ausente en los tres controladores debido a la respuesta sobreamortiguada. El predictor Smith rechaza la perturbación a diferencia de los otros controladores.

De acuerdo a la tabla 4.2, el efecto del controlador PID y Predictor Smith da un mismo tiempo de establecimiento de 15.8 minutos y el controlador PI da un tiempo de 13.7 minutos. El sobreimpulso que genera el PI es de $4.03\%$ mientras que tanto el PID y el Predictor Smith no generan sobreimpulso. El predictor Smith rechaza la perturbación a diferencia de los otros controladores.

De acuerdo a la tabla 4.3, el efecto del controlador PID y Predictor Smith da un mismo tiempo de establecimiento de 8.67 minutos y el controlador PI genera un tiempo de establecimiento de 20.5 minutos siendo mayor que lo requerido . El sobreimpulso que genera el PI es de $17.6\%$ mientras que tanto el PID y el Predictor Smith presenta un sobreimpulso de solo el 4.48%. El predictor Smith rechaza la perturbación a diferencia de los otros controladores.

De acuerdo a la tabla 4.4, el efecto del controlador PID y Predictor Smith da un mismo tiempo de establecimiento de 17.2 minutos y el controlador PI da 27.6 minutos, ninguno de los controladores cumple con los requerimientos. El sobreimpulso que genera el PI es de $30.2\%$ mientras que tanto el PID y el Predictor Smith presenta un sobreimpulso de 15.1% siendo estos valores tampoco El predictor Smith rechaza la perturbación a diferencia de los otros controladores.

# **BIBLIOGRAFÍA**

[15] **Alastruey, C., Gonzales, J.** (1995). “Resolución aproximada de sistemas lineales con retardos puntuales usando Series de Taylor”. 1-21.

[1] **Alfaro, V.** (2003). “ Nuevas aproximaciones del tiempo muerto para estudios de control”.1-13.

[22] **Ânstrom, K., Hang, C., Lim, B.** “A New Smith Predictor for Controlling a Process with an Integrator and Long Dead-Time”, IEEE Transactions on Automatic Control, Vol. 39, No. 2, pp. 343-345, Febrero 1994.

[8]  **Arroyave, D., Marquez, M., Gallego, D., and Pacheco, G.** (2010). “Evaluation and mineralogical characterization of biooxidation process in a continuos stirred tank reactor,” Dyna, vol. 77, no. 164, pp. 18–28.

[18] **Benítez, I., Rivas, R., Feliu, V. and Castillo, F.** “Predictor de Smith modificado mediante un modelo interno, robusto a perturbaciones externas no medibles.”

[5] **Cade, P., Aguilar, Rendón, R., Aguilar, J., Salinas, E., de la Cruz, F. and Sangerman, D.**  (2017). “Métodos cuantitativos, métodos cualitativos o su combinación en la investigación: un acercamiento en las ciencias sociales”. Revista Mexicana de Ciencias Agrícolas, 8, 1603–1617.

[33] **Castillo, S.** “Control automitico”, Educacion.

[13] **Claro, E.** (2018). “Diseño de un tanque mezclador para la Unidad Básica de Producción Recuperación de Amoníaco”.

[24] **Corzo, L.** (2019). "Diseño de un reactor de tanque agitado continuo con fines didácticos”, Accessed: Jul. 18, 2021.

[9] **Dong, Y., Yonghong, H., & Xu, G.** “Design of indoor swimming pool water temperature control system based on fuzzy controller and Smith predictor”. Proceedings of 2011 International Conference on Electronic and Mechanical Engineering and Information Technology, EMEIT 2011, 9, 4678–4681, 2011.

[37] **Feliu, V., Rivas, R. and Castillo, F.** (2013). “Simple Fraccional Order Controller Combined with a Smith Predictor for Temperature Control in a Steel Slab Reheating Furnace”. International Journal of Control, Automation, and Systems, 11(3), 533–544.

[7] **Gamboa, C., and Efraín, Gómez, B**. (2017). Escuela De Posgrado.Universidad Cesar Vallejo,Lima, Perú.

[32] **Giraldo, S., Flesch, R. and Normey, J.** “Multivariable Greenhouse Control Using the Filtered Smith Predictor.”

[11] **Gutiérrez, M.** (2015). “Tratamiento anaerobio de aguas residuales : Arranque , operación y seguimiento en una planta piloto a escala laboratorio", Máster en Ingeniería Ambiental, Universisdad de Valladolid, Valladolid, España, pp. 1–67.

[16] **Haibin, L., Jijian H., Yatao, S. and Shuang, L.** “Dynamic behavior analysis and time delay feedback control of gear pair system with backlash non-smooth characteristic” J. Vibroengineering, vol. 1u, Y9, no. 1, pp. 302–313, 2017.

[28] **Huba, M., Bistak, P., and Vranic, D.** “2-DoF IMC and smith-predictor-based control for stabilised unstable first order time delayed plants,” Mathematics, vol. 9, n. 9, May 2021.

[4] **Ivon, B., Raul, R., and Vicent, F.** (2017). “Predictor De Smith Modificado Para El Control De La Concentración En El Proceso De Producción De Medicamentos Inyectables”. Revista Mexicana de Ingeniería Química, 16(2), 635–649.

[21] **Khodadadi, H., and Dehghani, A.** “Fuzzy logic self-tuning PID controller design based on smith predictor for heating system”. International Conference on Control, Automation and Systems, 0(Iccas), 161–166, 2016.

[25] **Martínez, L.** (2006). "Simulación de una turbina radial mediante CFD FLUENT: caso de una turbina Rushton,” pp. 1–125.

[27] **Mejia, C.** (2019). "Diseño e implementación de cuatro esquemas de control modificados basados en el predictor de Smith en una tarjeta embebida, aplicados a dos modelos simulados que presentan retardo: un tanque de mezclado y un reactor de agitación continua (CSTR)". 1-172.

[35] **Molnar, T., Hajdu, D. and Insperg, T.** (2019). “The Smith predictor, the modified Smith predictor, and the finite spectrum assignment: A comparative study,” in Stability, Control and Application of Time-Delay Systems, Elsevier, pp. 209–226.

[36] **Neaca, M., Neaca, A., and Serban, T.** (2015). “Modified Smith Predictor for Long Time Thermal Processes”.

[19] **Normey, J. and Camacho, E.** (2007). “The Smith Predictor.”

[23]  **Ogata, K.** (1998). "Ingeniería de Control Moderna", Prentice Hall, pp. 217, 669-672, 813-826.

[31**] Orozco, G.** (1997). “La Investigación en comunicación desde la perspectiva Cualitativa. Universidad Nacional de la Plata”. Instituto Mexicano para el Desarrollo Comunitario, AC. México. 157 p.

[14]  **Palencia, A.** (2010). “Estudio de Diferentes Estrategias de Control para un Tanque de Mezclado: PID, Control de Matriz Dinámica (DMC) y Lógica Difusa (FLC).,” PROSPECTIVA, vol. 8, no. 1.

[38] **Palmor, Z. and Blau, M.** (1994). “An auto-tuner for Smith dead time compensator”, International Journal of Control, vol. 60, pp. 117-135.

[12] **Peña, E., Pérez, A., Ander, M. and Sanchez, J.** (2008). “Modelado de un reactor químico tipo CSTR y evaluación del control predictivo aplicando Matlab-Simulink Modeling to a CSTR reactor and evaluation of a predictive control using Matlab-Simulink,” Ing. Uc, vol. 15, pp. 97–112.

[6] **Persman, G.** “Sinopsis: Métodos de recoleccion y análisis de datos en la evaluacion de Impacto”, Simtesis metodologica,CENTRO DE INVESTIGACIONES INNOCENTI DE UNICEF, 2014.

[26]  **Quispe, L.** (2017). “OPTIMIZACIÓN TÉCNICA DEL SISTEMA MECÁNICO DE AGITACIÓN DE LA SALMUERA EN EL PROCESAMIENTO DE LA ACEITUNA,” Historia Santiago., p. Historia Santiago.

[20] **Rincón, J.** (2017). “DISEÑO DE UNA MÁQUINA MEZCLADORA, AUTOMÁTICA, DE MATERIAS PRIMAS PARA LA ELABORACIÓN DE JABONES LÍQUIDOS, SUAVIZANTES Y DESENGRASANTES INDUSTRIALES, PARA LA EMPRESA QUÍMICOS ZOREL,”

[10]  **Rodriguez, E., Deco, C., Burzacca, L., Costa, S. and Bender, C.** (n.d). "Recolección y análisis de datos obtenidos de una red de sensores para el monitoreo de condiciones de Higiene y Seguridad del ambiente en entornos industriales”.

[30] **Said, N., Engelhart, M., Kirches, S., Körbel, S. and Holt, V.** “Applying mathematical optimization methods to an ACT-R instance-based learning model,” PLoS One, vol. 11, no. 7, Jul. 2016.

[3] **Sakr, A., El-Nagar, A., Mohammad, E. and Sharaf, M.**  "Fuzzy Smith Predictor for Networked Control Systems".

[2] **Sanchez, A.** (2017). ”Técnicas de mantenimiento predictivo. metodología de aplicación en las Organizaciones”.

[17] **Shokri-Ghaleh, H., Ganjefar, S., and Shahri, A.** “Robust iterative learning control for uncertain continuous-time system with input delay and random iteration-varying uncertainties” IET Control Theory Appl., 2021

[34] **Thuengesripan, S., Suksri, T., Numsomran, A., Kongratana, V. and Roengruen, P.** (2007). “Smith Predictor Design by C D M for Temperature Control System”. International Conference on Control, Automation and Systems, 4, 17–20.

[29**] Veronesi, M.** (2003). “Performance improvement of Smith Predictor Through Automatic Computation of Dead Time”, Yokogawa Italia, Industrial Automation Department, Technical Report, No 35.