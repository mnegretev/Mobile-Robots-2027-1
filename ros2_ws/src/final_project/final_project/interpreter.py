"""Interpret Spanish commands for the mobile service robot.

The interpreter does not talk to ROS. It only decides which skill can run
and the short sentence the robot should speak.
"""

import re
import unicodedata


# Goals sit in the free regions used as spawn areas in house_simul.
PLACES = {
    'entrada': {
        'aliases': ('entrada', 'origen', 'inicio'),
        'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'label': 'la entrada',
    },
    'sala': {
        'aliases': ('sala', 'sillon', 'sofa'),
        'x': -1.0, 'y': -2.6, 'yaw': -1.2, 'label': 'la sala',
    },
    'television': {
        'aliases': ('television', 'tele', 'tv'),
        'x': -0.4, 'y': -4.2, 'yaw': -1.4, 'label': 'la televisión',
    },
    'cocina': {
        'aliases': ('cocina',),
        'x': 6.0, 'y': -3.0, 'yaw': 0.4, 'label': 'la cocina',
    },
    'refrigerador': {
        'aliases': ('refrigerador', 'refri', 'nevera'),
        'x': 7.0, 'y': -1.8, 'yaw': 0.3, 'label': 'el refrigerador',
    },
    'recamara': {
        'aliases': ('recamara', 'cuarto', 'dormitorio', 'cama'),
        'x': -7.6, 'y': -0.1, 'yaw': 1.57, 'label': 'la recámara',
    },
    'comedor': {
        'aliases': ('comedor',),
        'x': 1.6, 'y': 3.5, 'yaw': 1.57, 'label': 'el comedor',
    },
    'balcon': {
        'aliases': ('balcon',),
        'x': -0.4, 'y': 3.6, 'yaw': 2.5, 'label': 'el balcón',
    },
    'coca': {
        'aliases': ('coca', 'coca cola', 'cocacola', 'refresco'),
        'x': 4.8, 'y': -4.3, 'yaw': -1.2, 'label': 'la coca cola',
    },
    'mostaza': {
        'aliases': ('mostaza',),
        'x': 6.2, 'y': -1.7, 'yaw': 1.4, 'label': 'la mostaza',
    },
    'cheezit': {
        'aliases': ('cheezit', 'galleta', 'galletas'),
        'x': 5.2, 'y': -1.7, 'yaw': 1.3, 'label': 'las galletas',
    },
}

ARM_POSES = {
    'home': {
        'aliases': ('home', 'reposo', 'inicial', 'cero'),
        'joints': (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        'label': 'de reposo',
    },
    'navegacion': {
        'aliases': ('navegacion', 'guardar', 'recoger'),
        'joints': (0.0, -1.66, -0.72, 0.0, 1.17, 0.0),
        'label': 'de navegación',
    },
    'mirar': {
        'aliases': ('mirar', 'observa', 'camara'),
        'joints': (0.0, -0.9, -0.4, 0.0, 1.2, 0.0),
        'label': 'para mirar',
    },
    'estirar': {
        'aliases': ('estirar', 'estira', 'extender', 'extiende', 'adelante'),
        'joints': (0.0, -0.45, -0.35, 0.0, 0.75, 0.0),
        'label': 'estirada',
    },
    'saludar': {
        'aliases': ('saludar', 'saluda', 'saludo'),
        'joints': (0.9, -0.7, -0.35, 0.5, 1.0, 0.4),
        'label': 'de saludo',
    },
    'arriba': {
        'aliases': ('arriba', 'levanta', 'levantar', 'sube', 'subir'),
        'joints': (0.0, -0.35, -0.2, 0.0, 0.45, 0.0),
        'label': 'arriba',
    },
}

# xarm6 with the limited flag used by the house simulation.
JOINT_LIMITS = (
    (-3.10, 3.10),
    (-2.05, 2.09),
    (-3.10, 0.19),
    (-3.10, 3.10),
    (-1.69, 3.10),
    (-3.10, 3.10),
)

_GO = (
    've a', 've al', 'vete', 'navega', 'dirigete', 'llevale', 'llevame',
    'acercate', 'camina', 'muevete', 'quiero ir', 'ir a', 'anda a',
    'anda hacia',
)
_LOOK = (
    'reconoce', 'reconocer', 'busca', 'buscar', 'que ves', 'que hay',
    'identifica', 'observa', 'que objeto', 'que objetos',
)
_ARM = ('brazo', 'manipulador', 'pinza', 'gripper', 'articulacion')
_OBJECTS = (
    ('coca', ('coca', 'coca cola', 'cocacola', 'refresco')),
    ('mostaza', ('mostaza',)),
    ('cheezit', ('cheezit', 'galleta', 'galletas')),
    ('silla', ('silla', 'sillas')),
    ('mesa', ('mesa',)),
    ('persona', ('persona', 'alguien', 'gente')),
    ('taza', ('taza', 'tazon')),
    ('botella', ('botella',)),
    ('cama', ('cama',)),
    ('sofa', ('sofa', 'sillon')),
    ('television', ('television', 'tele', 'tv')),
)
_IMPOSSIBLE = (
    (('vuela', 'volar', 'despega', 'despegar'),
     'No puedo volar. Solo me desplazo por el piso.'),
    (('abre la puerta', 'abrir la puerta', 'cierra la puerta', 'abrir puertas'),
     'No puedo abrir ni cerrar puertas.'),
    (('cocinar', 'cocina un', 'pastel', 'receta'),
     'No puedo cocinar.'),
    (('escalera', 'escaleras'),
     'No puedo subir ni bajar escaleras.'),
    (('agarra', 'agarrar', 'toma la', 'tomar la', 'coge', 'sujeta',
      'traeme', 'trae la', 'traer', 'lleva la', 'llevate'),
     'No puedo tomar ni traer objetos. Sí puedo reconocerlos.'),
    (('bailar', 'baila', 'corre', 'correr', 'salta', 'saltar'),
     'No puedo hacer ese movimiento con la base.'),
    (('marte', 'luna', 'al espacio', 'al cielo'),
     'No puedo salir de la casa simulada.'),
)


def _fold(text):
    raw = text.lower().strip()
    raw = ''.join(
        ch for ch in unicodedata.normalize('NFD', raw)
        if unicodedata.category(ch) != 'Mn'
    )
    raw = re.sub(r'(\d),(\d)', r'\1.\2', raw)
    raw = re.sub(r'[^a-z0-9.\- ]+', ' ', raw)
    return re.sub(r'\s+', ' ', raw).strip()


def _has(text, phrases):
    return any(phrase in text for phrase in phrases)


def _word(text, alias):
    return re.search(r'(?<!\w)' + re.escape(alias) + r'(?!\w)', text) is not None


def _find_named(text, table, alias_key='aliases'):
    hits = []
    for name, spec in table.items():
        for alias in spec[alias_key]:
            match = re.search(r'(?<!\w)' + re.escape(alias) + r'(?!\w)', text)
            if match:
                hits.append((match.start(), -len(alias), name))
                break
    if not hits:
        return None
    pivots = [m.start() for m in re.finditer(r'(?<!\w)(en|a|al|hacia)(?!\w)', text)]
    if pivots:
        after = [hit for hit in hits if hit[0] > pivots[-1]]
        if after:
            after.sort()
            return after[0][2]
    hits.sort()
    return hits[0][2]


def _find_object(text):
    found = None
    best = -1
    for name, aliases in _OBJECTS:
        for alias in aliases:
            match = re.search(r'(?<!\w)' + re.escape(alias) + r'(?!\w)', text)
            if match and match.start() >= best:
                best = match.start()
                found = name
    return found


def _coords(text):
    numbers = re.findall(r'-?\d+(?:\.\d+)?', text)
    if len(numbers) < 2:
        return None
    return float(numbers[0]), float(numbers[1])


def _reject(say, try_llm=False):
    return {
        'skill': 'reject',
        'say': say,
        'try_llm': try_llm,
        'place': None,
        'object': None,
        'pose': None,
    }


def _place_cmd(name, then=None, obj=None, say=None):
    spec = PLACES[name]
    return {
        'skill': 'navigate',
        'place': name,
        'x': spec['x'],
        'y': spec['y'],
        'yaw': spec['yaw'],
        'label': spec['label'],
        'then': then,
        'object': obj,
        'say': say or ('Voy hacia %s.' % spec['label']),
        'try_llm': False,
        'pose': None,
    }


def _arm_command(text):
    pose = _find_named(text, ARM_POSES)
    if pose:
        spec = ARM_POSES[pose]
        return {
            'skill': 'manipulate',
            'mode': 'named',
            'pose': pose,
            'say': 'Muevo el brazo a la posición %s.' % spec['label'],
            'try_llm': False,
        }
    if _word(text, 'izquierda'):
        return {
            'skill': 'manipulate', 'mode': 'relative', 'joint': 0,
            'delta': 0.6, 'say': 'Giro el brazo a la izquierda.',
            'try_llm': False, 'pose': None,
        }
    if _word(text, 'derecha'):
        return {
            'skill': 'manipulate', 'mode': 'relative', 'joint': 0,
            'delta': -0.6, 'say': 'Giro el brazo a la derecha.',
            'try_llm': False, 'pose': None,
        }
    numbers = re.findall(r'-?\d+(?:\.\d+)?', text)
    if len(numbers) >= 6:
        vals = [float(n) for n in numbers[:6]]
        return {
            'skill': 'manipulate', 'mode': 'joints', 'joints': vals,
            'say': 'Intento llevar el brazo a esas articulaciones.',
            'try_llm': False, 'pose': None,
        }
    if len(numbers) >= 3:
        vals = [float(n) for n in numbers[:3]]
        return {
            'skill': 'manipulate', 'mode': 'cartesian',
            'x': vals[0], 'y': vals[1], 'z': vals[2],
            'say': 'Intento llevar el brazo a esa pose.',
            'try_llm': False, 'pose': None,
        }
    return {
        'skill': 'manipulate', 'mode': 'named', 'pose': 'navegacion',
        'say': 'Muevo el brazo a la posición de navegación.',
        'try_llm': False,
    }


def _faq(text):
    if _has(text, ('como te llamas', 'quien eres', 'que eres', 'tu nombre')):
        return 'Soy el robot de servicio del curso de robots móviles.', None
    if _has(text, ('que puedes', 'que sabes hacer', 'cuales son tus', 'ayuda')):
        return (
            'Puedo ir a un lugar de la casa, reconocer objetos, '
            'mover el brazo y contestar preguntas.',
            None,
        )
    if _has(text, ('donde estas', 'donde te encuentras', 'tu posicion', 'tu ubicacion')):
        return 'Déjame ver dónde estoy.', 'pose'
    if _has(text, ('que hora', 'que horas')):
        return 'Déjame ver la hora.', 'time'
    if _has(text, ('que dia', 'que fecha')):
        return 'Déjame ver la fecha.', 'date'
    if _has(text, ('quien es el profesor', 'marco negrete', 'quien da el curso')):
        return 'El profesor del curso es Marco Negrete, de la FI UNAM.', None
    if _has(text, ('que curso', 'de que clase')):
        return 'Este software es del curso de robots móviles, FI UNAM, dos mil veintisiete.', None
    place = _find_named(text, PLACES)
    if place and _has(text, ('donde esta', 'donde queda', 'donde se encuentra')):
        return 'Puedo llevarte. Dime ve hacia %s.' % PLACES[place]['label'], None
    return None


def interpret(text):
    """Return a skill dictionary for one spoken or written utterance."""
    raw = _fold(text or '')
    if len(raw) < 2:
        return _reject('No escuché una orden clara.')

    if raw in ('alto', 'detente', 'para', 'stop') or raw.startswith('deten'):
        return {
            'skill': 'stop',
            'say': 'Me detengo.',
            'try_llm': False,
            'pose': None,
            'place': None,
            'object': None,
        }

    if _has(raw, _ARM):
        return _arm_command(raw)

    place = _find_named(raw, PLACES)
    obj = _find_object(raw)
    wants_look = _has(raw, _LOOK)
    wants_go = _has(raw, _GO)

    if wants_look:
        if place and (wants_go or ' en ' in (' ' + raw + ' ')):
            return _place_cmd(place, then='recognize', obj=obj)
        if obj is None:
            spoken = 'Voy a mirar qué objetos hay.'
        elif obj in PLACES:
            spoken = 'Voy a buscar %s.' % PLACES[obj]['label']
        else:
            spoken = 'Voy a buscar %s.' % obj
        return {
            'skill': 'recognize',
            'object': obj,
            'say': spoken,
            'try_llm': False,
            'pose': 'mirar',
            'place': None,
        }

    if wants_go:
        if place:
            return _place_cmd(place)
        point = _coords(raw)
        if point is not None:
            return {
                'skill': 'navigate',
                'place': None,
                'x': point[0],
                'y': point[1],
                'yaw': 0.0,
                'label': 'el punto %.1f %.1f' % point,
                'then': None,
                'object': None,
                'say': 'Voy al punto %.1f, %.1f.' % point,
                'try_llm': False,
                'pose': None,
            }
        return _reject('No conozco ese lugar, así que no puedo ir.')

    for keys, message in _IMPOSSIBLE:
        if _has(raw, keys):
            return _reject(message)

    if '?' in (text or '') or _has(raw, (
        'que ', 'quien', 'cual', 'como ', 'donde', 'por que', 'cuanto',
        'sabes', 'dime', 'puedes decir',
    )):
        faq = _faq(raw)
        if faq:
            say, dynamic = faq
            return {
                'skill': 'answer',
                'say': say,
                'dynamic': dynamic,
                'try_llm': dynamic is None and say.startswith('No tengo'),
                'pose': None,
                'place': None,
                'object': None,
            }
        return {
            'skill': 'answer',
            'say': 'No tengo esa información ahora.',
            'dynamic': None,
            'try_llm': True,
            'pose': None,
            'place': None,
            'object': None,
        }

    return _reject(
        'No entendí. Puedo ir a un lugar, reconocer objetos, '
        'mover el brazo o contestar preguntas.',
        try_llm=True,
    )
