# Control de posición de un motor de corriente continua con MATLAB y Simulink

Modelado y control PID de un motor de corriente continua que acciona una barra giratoria vinculada a un resorte y un amortiguador. Este proyecto académico combina un modelo electromecánico, análisis en el dominio de la frecuencia y simulación del control de posición en lazo cerrado.

Desarrollado por **Facundo Aguilera Van Cauwlaert** para la materia **Control of Mechanical Systems**, semestre **2025/2026**.

**[English version](README.md)** · **[Informe técnico en inglés](Report%204%20-%20DC%20Motor.pdf)**

## Descripción del proyecto

El objetivo es regular la posición angular de una carga mecánica mediante la tensión de armadura del motor. El modelo considera la resistencia e inductancia de armadura, el par del motor, la fuerza contraelectromotriz, la inercia mecánica, el amortiguamiento, la rigidez del resorte y los efectos linealizados de la gravedad.

El estudio incluye:

- Obtención de la función de transferencia entre tensión y posición angular.
- Análisis de los polos de la planta y de los polos y ceros en lazo abierto.
- Ajuste del PID mediante análisis de respuesta en frecuencia y lugar de las raíces, documentado en el informe.
- Evaluación de la respuesta al escalón en lazo cerrado e implementación del controlador en Simulink.

## Contenido del repositorio

| Archivo | Función |
| --- | --- |
| [`DCMotor.m`](DCMotor.m) | Define los parámetros, construye las funciones de transferencia de la planta y del PID, evalúa polos e indicadores de respuesta al escalón, grafica el diagrama de Bode con asíntotas y ejecuta Simulink. |
| [`simulationDCMotor.slx`](simulationDCMotor.slx) | Modelo de control de posición con realimentación unitaria y ramas proporcional, integral y derivativa separadas. |
| [`asymp.m`](asymp.m) | Función auxiliar incluida para graficar diagramas de Bode con asíntotas de magnitud y fase. |
| [`Report 4 - DC Motor.pdf`](Report%204%20-%20DC%20Motor.pdf) | Desarrollo del modelo, comparación de ajustes del controlador e indicadores de respuesta obtenidos en el estudio. |

## Modelo matemático

Para la coordenada angular $\theta$, los parámetros mecánicos equivalentes son

$$
J_{\mathrm{eq}}=J+\frac{mL^2}{4},\qquad
b_{\mathrm{eq}}=rL^2,\qquad
k_{\mathrm{eq}}=kL^2-\frac{mgL}{2}.
$$

Las ecuaciones mecánica y eléctrica linealizadas implementadas en el código son

$$
J_{\mathrm{eq}}\ddot{\theta}+b_{\mathrm{eq}}\dot{\theta}+k_{\mathrm{eq}}\theta=k_\phi i_a,
$$

$$
L_a\dot{i}_a+R_a i_a=V_a-k_\phi\dot{\theta}.
$$

Se utiliza el mismo valor numérico de $k_\phi$ para las constantes de par y de fuerza contraelectromotriz, con unidades coherentes del SI. La función de transferencia entre tensión y posición angular es

$$
G(s)=\frac{\Theta(s)}{V_a(s)}=
\frac{k_\phi}{(J_{\mathrm{eq}}s^2+b_{\mathrm{eq}}s+k_{\mathrm{eq}})(L_as+R_a)+k_\phi^2s}.
$$

El controlador y la función de transferencia en lazo cerrado son

$$
R(s)=K_p\left(1+T_ds+\frac{1}{T_is}\right),\qquad
T(s)=\frac{R(s)G(s)}{1+R(s)G(s)}.
$$

Estas ecuaciones corresponden a la implementación de `DCMotor.m`; la ecuación eléctrica impresa en el informe contiene inconsistencias tipográficas.

## Parámetros predeterminados

| Parámetro | Valor |
| --- | ---: |
| Masa de la carga, `m` | 80 kg |
| Parámetro de inercia, `J` | 20 kg·m² |
| Longitud de la barra, `L` | 1 m |
| Rigidez del resorte, `k` | 500 N/m |
| Coeficiente de amortiguamiento, `r` | 25 N·s/m |
| Aceleración de la gravedad, `g` | 9.81 m/s² |
| Inductancia de armadura, `La` | 0.001 H |
| Resistencia de armadura, `Ra` | 0.1 Ω |
| Constante de par del motor, `k_phi` | 0.0001 N·m/A |
| Ganancia proporcional, `kp` | 1000 |
| Tiempo integral, `Ti` | 1 s |
| Tiempo derivativo, `Td` | 0.5 s |
| Ángulo de referencia, `theta_ref` | 1 rad |

Con estos valores se obtiene $J_{\mathrm{eq}}=40$ kg·m², $b_{\mathrm{eq}}=25$ N·m·s/rad y $k_{\mathrm{eq}}=107.6$ N·m/rad. En la forma paralela del PID, $K_i=K_p/T_i=1000$ y $K_d=K_pT_d=500$.

## Requisitos

- MATLAB y Simulink. El modelo fue guardado en **R2025b**; para abrirlo en versiones anteriores puede ser necesario exportarlo a una versión previa.
- Control System Toolbox para funciones como `tf`, `series`, `feedback`, `pole`, `zero`, `stepinfo` y `bode`.

Todos los archivos del repositorio deben permanecer en la misma carpeta de trabajo. El modelo suministrado no utiliza bloques de Simscape.

## Ejecución del proyecto

Descargar o clonar el repositorio:

```bash
git clone https://github.com/facuavc/dc-motor-control-matlab-simulink.git
cd dc-motor-control-matlab-simulink
```

Seleccionar esta carpeta como carpeta actual (*Current Folder*) de MATLAB y ejecutar:

```matlab
DCMotor
```

El script borra las variables del espacio de trabajo y cierra las figuras existentes. Luego crea las funciones de transferencia, muestra los resultados del análisis, grafica el diagrama de Bode en lazo abierto y ejecuta `simulationDCMotor.slx`.

Para visualizar el diagrama de bloques:

```matlab
open_system('simulationDCMotor.slx')
```

Ejecutar el script antes de simular el modelo de manera independiente: sus bloques dependen de variables del espacio de trabajo como `G_num`, `G_den`, `kp`, `Ti`, `Td` y `theta_ref`.

### Graficar la posición simulada

Una vez terminada la simulación, se puede utilizar directamente la serie temporal registrada:

```matlab
figure;
plot(sim_out.position.Time, sim_out.position.Data, 'LineWidth', 1.5);
hold on;
yline(theta_ref, '--', 'Referencia');
grid on;
xlabel('Tiempo [s]');
ylabel('Posición angular [rad]');
title('Respuesta de posición en lazo cerrado');
legend('Posición', 'Referencia', 'Location', 'best');
```

Si la última línea de `DCMotor.m` produce un error de nombre de propiedad, reemplazar `sim_out.position.data` por la propiedad estándar de los objetos timeseries: `sim_out.position.Data`.

### Duración de la simulación y gráficos adicionales

El modelo guardado tiene un tiempo final fijo de **600 s**. Aunque el script define `sT = 1000`, la llamada actual a `sim` no transmite ese valor al modelo. Para simular explícitamente durante `sT` segundos, reemplazar dicha llamada por:

```matlab
sim_out = sim('simulationDCMotor.slx', 'StopTime', num2str(sT));
```

Los comandos de márgenes de estabilidad, Nyquist y lugar de las raíces están comentados en el script. Para utilizarlos, descomentarlos o ejecutar lo siguiente después de crear las funciones de transferencia:

```matlab
figure; margin(GH);
figure; nyquist(GH);
figure; rlocus(GH);
figure; step(Ls); grid on;
```

Para modificar el sistema o el controlador, editar los parámetros en `DCMotor.m` antes de ejecutar nuevamente el script completo. Los valores asignados manualmente en el espacio de trabajo serán sobrescritos por el script.

## Resultados documentados en el estudio

Para el ajuste seleccionado, el informe técnico presenta los siguientes resultados:

| Indicador | Valor informado |
| --- | ---: |
| Tiempo de subida | 238.116 s |
| Tiempo de establecimiento | 423.156 s |
| Sobreimpulso | Sin sobreimpulso informado |
| Error de seguimiento en régimen permanente | 0 rad |

Estos valores provienen del informe y no de una nueva ejecución realizada para preparar este README. El script utiliza `stepinfo(Ls)` con los umbrales de respuesta predeterminados de MATLAB.

## Alcance y limitaciones

El proyecto es un estudio académico de simulación en tiempo continuo. La gravedad se linealiza alrededor de $\theta=0$, por lo que la referencia de 1 rad debe interpretarse como una prueba del modelo lineal y no como una validación del comportamiento físico para ángulos grandes.

El controlador utiliza una derivada ideal. El modelo no incluye filtrado de la acción derivativa, saturación de tensión o corriente, mecanismos anti-windup, ruido de sensores, muestreo digital ni validación experimental. Estos aspectos deberían incorporarse antes de una implementación práctica.
