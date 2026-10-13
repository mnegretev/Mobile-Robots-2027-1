#!/usr/bin/env bash
# Sube la Práctica 2 al repositorio de la materia (rama actual).
# Ejecutar con `bash subir_a_github.sh` desde cualquier lugar DENTRO del repositorio
# (en el host o en el contenedor). No toca nada más que esta carpeta y los dos nodos.
set -e
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAIZ="$(git -C "$AQUI" rev-parse --show-toplevel 2>/dev/null)" || { echo "Esta carpeta no está dentro de un repositorio git."; exit 1; }
cd "$RAIZ"
RAMA="$(git branch --show-current)"
echo "Repositorio: $RAIZ"
echo "Rama:        $RAMA"
echo "Remoto:      $(git remote get-url origin)"
if [ "$RAMA" != "rosario_carrasco" ]; then
  read -r -p "La rama actual no es rosario_carrasco. ¿Continuar de todos modos? [s/N] " r
  [[ "$r" =~ ^[sS]$ ]] || exit 1
fi
PF="$(realpath --relative-to="$RAIZ" "$AQUI/../path_follower/path_follower")"
git add -- "$(realpath --relative-to="$RAIZ" "$AQUI")" "$PF/pure_pursuit.py" "$PF/stanley.py"
echo
echo "Esto se va a subir (A = nuevo, M = modificado):"
git status --short
echo
read -r -p "¿Hacer commit y push? [s/N] " r
[[ "$r" =~ ^[sS]$ ]] || { echo "Cancelado. Lo agregado queda en el área de preparación (git restore --staged . para deshacerlo)."; exit 0; }
git commit -m "Práctica 2: seguimiento de rutas (pure pursuit y Stanley), experimentos, análisis y reporte"
git push -u origin "$RAMA"
echo "Listo."
