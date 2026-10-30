#!/usr/bin/env bash
# Baja los resultados de una corrida de simular.yml a build/sim/<run-id>/, para leerlos
# con el agente. build/ está en .gitignore: no se commitea.
#
# Uso, desde la raíz del repo:
#   bash notes/AlsinaBautista/bajar_corrida.sh            # tu última corrida
#   bash notes/AlsinaBautista/bajar_corrida.sh <run-id>   # una corrida en particular
#
# Necesita gh con sesión iniciada. Para los logs (equipo.log, juez.log, ...), Tailscale
# prendido: viven en el servidor, no en el artifact de GitHub.
set -euo pipefail

REPO=Emiliano-Aguilera/jar_challenge
SERVER=https://server-trx50-ubuntu.tail4dd159.ts.net/runs

if [ $# -ge 1 ]; then
  RUN_ID=$1
else
  USER_LOGIN=$(gh api user --jq .login)
  RUN_ID=$(gh run list -R "$REPO" --workflow simular.yml --user "$USER_LOGIN" --limit 1 \
    --json databaseId --jq '.[0].databaseId')
fi
echo "Corrida $RUN_ID: espero a que termine..."
gh run watch "$RUN_ID" -R "$REPO" --exit-status > /dev/null || echo "La corrida terminó con error; bajo lo que haya."

DEST=build/sim/$RUN_ID
rm -rf "$DEST"
mkdir -p "$DEST"
gh run download "$RUN_ID" -R "$REPO" -D "$DEST"   # un artifact lote-<id> por mapa

for lote_dir in "$DEST"/lote-*/; do
  lote=$(basename "$lote_dir")
  lote=${lote#lote-}
  for run_dir in "$lote_dir"corridas/*/; do
    name=$(basename "$run_dir")
    for f in equipo.log juez.log eventos.jsonl choques.json; do
      curl -fsS "$SERVER/$lote/corridas/$name/$f" -o "$run_dir$f" \
        || echo "No pude bajar $f de $name (¿Tailscale prendido?)"
    done
  done
done
echo "Listo: $DEST"
