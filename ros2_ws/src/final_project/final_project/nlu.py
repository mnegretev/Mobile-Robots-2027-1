"""
Comprensión de lenguaje natural (NLU) para el proyecto final.

Convierte una frase en español (salida de Whisper) en una o varias
"intenciones" que el nodo voice_commander sabe ejecutar.
No depende de ROS, así que se puede probar con:  python3 -m final_project.nlu
"""
import re
import unicodedata

INTERROGATIVES = ("que", "como", "cual", "cuales", "donde", "cuando", "quien",
                  "quienes", "cuanto", "cuantos", "cuanta", "por que", "porque",
                  "sabes", "puedes decirme", "dime")

NAV_VERBS = r"(ve|vete|ir|vaya|anda|camina|dirigete|muevete|navega|avanza|llevame|" \
            r"acude|desplazate|regresa|vuelve|regresate|ven)"
GRASP_VERBS = r"(toma|tomes|agarra|agarres|recoge|recojas|sujeta|coge|levanta|traeme|trae|traer|tomar|agarrar|recoger)"
FIND_VERBS = r"(busca|buscar|encuentra|encontrar|reconoce|reconocer|identifica|identificar|localiza|detecta|ves|mira si|observa)"
ARM_WORDS = r"(brazo|manipulador|mano|pinza|gripper|garra)"

STOP_WORDS = r"^(alto|detente|para|parate|stop|cancela|cancelar|espera|quieto|quedate)\b"

# Acciones que el robot claramente no puede hacer.
IMPOSSIBLE = [
    (r"\b(vuela|volar|vueles)\b", "volar"),
    (r"\b(salta|saltar|brinca)\b", "saltar"),
    (r"\b(escaleras?|sube al segundo piso)\b", "subir escaleras"),
    (r"\b(nada|nadar)\b(?! mas)", "nadar"),
    (r"\b(cocina(me)?|preparame|prepara|cocinar|hazme de comer)\b(?! *$)", "cocinar"),
    (r"\b(abre|abrir|cierra|cerrar) (la )?(puerta|ventana|refrigerador|refri)\b", "abrir o cerrar puertas"),
    (r"\b(limpia|barre|trapea|aspira|lava)\b", "hacer limpieza"),
    (r"\b(baila|bailar|canta|cantar)\b", "bailar o cantar"),
    (r"\b(corre|correr|carrera)\b", "correr"),
    (r"\b(maneja|conduce|manejar)\b", "manejar un auto"),
]


def normalize(text):
    """Minúsculas, sin acentos ni signos de puntuación."""
    t = unicodedata.normalize("NFD", text.lower().replace("ñ", "\x00"))
    t = "".join(c for c in t if unicodedata.category(c) != "Mn").replace("\x00", "ñ")
    t = re.sub(r"[^a-z0-9ñ ]+", " ", t)
    return re.sub(r"\s+", " ", t).strip()


class NLU:
    def __init__(self, knowledge, robot_name="robot"):
        self.locations = knowledge.get("locations", {})
        self.objects = knowledge.get("objects", {})
        self.robot_name = normalize(robot_name)
        self.loc_aliases = self._build_aliases(self.locations)
        self.obj_aliases = self._build_aliases(self.objects)

    @staticmethod
    def _build_aliases(table):
        out = []
        for key, info in table.items():
            for a in [key.replace("_", " ")] + list(info.get("aliases", [])):
                out.append((normalize(a), key))
        out.sort(key=lambda p: -len(p[0]))  # el alias más largo gana ("lata verde" antes que "lata")
        return out

    @staticmethod
    def _find_alias(text, aliases):
        for alias, key in aliases:
            if re.search(r"\b" + re.escape(alias) + r"\b", text):
                return key
        return None

    @staticmethod
    def _target_after(text, verb_regex):
        """Extrae las palabras después del verbo (para decir qué no conoce)."""
        m = re.search(verb_regex + r"\s+(?:(?:a|al|hacia|hasta|por|para)\s+)?(?:(?:la|el|los|las|un|una|mi|me)\s+)?(.+)$", text)
        if not m:
            return ""
        target = m.group(m.lastindex).strip()
        target = re.sub(r"^(la|el|los|las|un|una|me)\s+", "", target)
        return " ".join(target.split()[:4])

    # ------------------------------------------------------------------
    def parse(self, raw_text):
        """Devuelve una lista de intenciones (diccionarios)."""
        # Las comas se conservan como separador para las órdenes compuestas.
        text = normalize(raw_text.replace(",", " sepcoma "))
        names = ["robot"]
        if self.robot_name:
            names.insert(0, re.escape(self.robot_name))
        text = re.sub(r"^(?:oye |hola )?(?:" + "|".join(names) + r")\b(?: sepcoma)?\s*", "", text)
        text = re.sub(r"^(?:sepcoma\s*)+", "", text).strip()
        if not text.replace("sepcoma", "").strip():
            return []
        first = text.replace("sepcoma", " ").strip()
        is_question = "?" in raw_text or bool(re.match(r"^(" + "|".join(INTERROGATIVES) + r")\b", first))
        # Órdenes compuestas: "ve a la cocina y luego busca la mostaza"
        if not is_question:
            parts = [p.strip() for p in re.split(r"\b(?:y luego|y despues|luego|despues|y|sepcoma)\b", text) if p.strip()]
            if len(parts) > 1:
                intents = [self._parse_single(p, False) for p in parts]
                # Si al menos una parte se entendió, se ejecutan las partes en orden;
                # las que no se entiendan se reportan al usuario.
                if any(i["type"] not in ("unknown", "question") for i in intents):
                    return intents
        return [self._parse_single(re.sub(r"\s+", " ", text.replace("sepcoma", " ")).strip(), is_question)]

    def _parse_single(self, text, is_question):
        loc = self._find_alias(text, self.loc_aliases)
        obj = self._find_alias(text, self.obj_aliases)

        if re.search(STOP_WORDS, text):
            return {"type": "stop"}

        # --- Preguntas sobre el robot / visión --------------------------
        if re.search(r"\b(que|cuales|cuantos) (objetos )?(ves|hay|observas|detectas|reconoces|miras)\b", text) \
                or re.search(r"\b(describe|dime) lo que ves\b", text):
            if obj:
                return {"type": "find", "object": obj}
            return {"type": "describe_scene"}
        if re.search(r"\b" + FIND_VERBS + r"\b", text) and not re.search(r"\b(donde esta|donde se encuentra)\b", text):
            if obj:
                return {"type": "find", "object": obj}
            target = self._target_after(text, r"\b" + FIND_VERBS + r"\b")
            if target and not re.search(r"\b(objetos?|algo|cosas?)\b", target):
                return {"type": "find_unknown", "name": target}
            return {"type": "describe_scene"}

        if re.search(r"\b(como te llamas|cual es tu nombre|quien eres|tu nombre)\b", text):
            return {"type": "question", "kind": "name"}
        if re.search(r"\b(que puedes hacer|que sabes hacer|cuales son tus habilidades|que haces|ayuda)\b", text):
            return {"type": "question", "kind": "abilities"}
        if re.search(r"\b(donde estas|en donde estas|donde te encuentras|cual es tu posicion|tu ubicacion)\b", text):
            return {"type": "question", "kind": "where_am_i"}
        if re.search(r"\b(que hora es|que horas son|la hora)\b", text):
            return {"type": "question", "kind": "time"}
        if re.search(r"\b(que dia es|que fecha es|la fecha)\b", text):
            return {"type": "question", "kind": "date"}
        if re.search(r"\b(que lugares conoces|a donde puedes ir|que lugares|que cuartos)\b", text):
            return {"type": "question", "kind": "known_places"}
        if re.search(r"\b(que objetos conoces|que cosas conoces|que puedes tomar)\b", text):
            return {"type": "question", "kind": "known_objects"}
        if re.search(r"\b(como estas|que tal)\b", text):
            return {"type": "question", "kind": "how_are_you"}
        if re.search(r"\b(donde esta|donde estan|donde se encuentra|donde hay|donde dejaste)\b", text):
            if obj:
                return {"type": "question", "kind": "object_location", "object": obj}
            if loc:
                return {"type": "question", "kind": "place_info", "location": loc}
            return {"type": "question", "kind": "object_location_unknown",
                    "name": self._target_after(text, r"\b(esta|estan|encuentra|hay)\b")}

        # --- Acciones imposibles -------------------------------------
        for pattern, action in IMPOSSIBLE:
            if re.search(pattern, text) and not (loc and re.search(r"\b" + NAV_VERBS + r"\b", text)):
                return {"type": "impossible", "action": action}

        # --- Manipulador ---------------------------------------------
        if re.search(r"\b" + ARM_WORDS + r"\b", text) or re.search(r"\b(saluda|saludame|saludar|despidete)\b", text):
            return self._parse_arm(text)

        # --- Tomar / traer objetos -----------------------------------
        if re.search(r"\b" + GRASP_VERBS + r"\b", text):
            bring = bool(re.search(r"\b(traeme|trae|traer|traigas|dame)\b", text))
            if obj:
                return {"type": "grasp", "object": obj, "bring": bring}
            return {"type": "grasp_unknown", "name": self._target_after(text, r"\b" + GRASP_VERBS + r"\b")}

        # --- Navegación ----------------------------------------------
        if re.search(r"\b" + NAV_VERBS + r"\b", text):
            if loc:
                return {"type": "navigate", "location": loc}
            if obj:  # "ve a donde esta la mostaza"
                return {"type": "navigate", "location": self.objects[obj]["location"]}
            if re.search(r"\b(regresa|vuelve|regresate)\b", text) and not re.search(r"\b(a|al|hacia)\b", text):
                return {"type": "navigate", "location": "inicio"}
            return {"type": "navigate_unknown", "name": self._target_after(text, r"\b" + NAV_VERBS + r"\b")}

        if is_question:
            return {"type": "question", "kind": "general", "text": text}
        return {"type": "unknown", "text": text}

    def _parse_arm(self, text):
        if re.search(r"\b(saluda|saludame|saludar|despidete|agita)\b", text):
            return {"type": "arm", "action": "wave"}
        if re.search(r"\b(abre|abrir|suelta|soltar)\b", text):
            return {"type": "arm", "action": "gripper_open"}
        if re.search(r"\b(cierra|cerrar|aprieta)\b", text):
            return {"type": "arm", "action": "gripper_close"}
        if re.search(r"\b(levanta|sube|arriba|alza|eleva)\b", text):
            return {"type": "arm", "action": "pose", "pose": "levantar"}
        if re.search(r"\b(estira|extiende|adelante|al frente)\b", text):
            return {"type": "arm", "action": "pose", "pose": "estirar"}
        if re.search(r"\b(baja|abajo|descansa|guarda|recoge|inicio|home|reposo|normal)\b", text):
            return {"type": "arm", "action": "pose", "pose": "home"}
        if re.search(r"\b(mira|apunta|observa)\b", text):
            return {"type": "arm", "action": "pose", "pose": "mirar"}
        if re.search(r"\b(mueve|muevelo|mover)\b", text):
            return {"type": "arm", "action": "pose", "pose": "estirar"}
        return {"type": "arm_unknown", "text": text}


if __name__ == "__main__":
    import sys
    import yaml
    path = "config/knowledge.yaml"
    if len(sys.argv) > 1:
        path = sys.argv[1]
    nlu = NLU(yaml.safe_load(open(path)), "Lira")
    print("Escribe una orden (Ctrl+D para salir):")
    for line in sys.stdin:
        print(nlu.parse(line))
