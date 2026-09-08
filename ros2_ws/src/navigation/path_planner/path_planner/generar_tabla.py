# generar_tablas.py
import pandas as pd

df = pd.read_csv("experimentos_rrt.csv")

resumen = df.groupby(["Goal_X", "Goal_Y", "Epsilon", "N"]).agg(
    Intentos=("Exito", "count"),
    Exitos=("Exito", lambda x: (x == True).sum()),
    Tasa_Exito=("Exito", lambda x: f"{(x.sum()/len(x))*100:.1f}%"),
    Tiempo_Prom_ms=("Tiempo_s", lambda x: f"{(x.mean()*1000):.1f} ms")
).reset_index()

# Imprimir en consola con formato Markdown
print(resumen.to_markdown(index=False))

# Opcional: exportar directo a Excel para tu reporte
resumen.to_excel("tabla_reporte_actividad_7.xlsx", index=False)
