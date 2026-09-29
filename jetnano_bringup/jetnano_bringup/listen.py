# Copyright 2026 stevej52
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Listen: Rosie understands a few words.

    ros2 run jetnano_bringup listen                  (in ~/venv-voice: see
                                                      robot-environment/scripts/install_voice.sh)
    ros2 topic echo /speech/text                     what she made of what she heard
    ros2 topic pub --once /speech/type std_msgs/msg/String "{data: 'Rosie, what is the weather?'}"
    ros2 run jetnano_bringup listen --file a.wav ... try the recognizer on recordings
    ros2 run jetnano_bringup listen --news all       print a briefing (world|us|local|weather|markets)

Takes the microphone stream the ears publish (``sound/audio``, 16 kHz), cuts
it into utterances with a voice activity detector and turns each into text
with a small speech-to-text model on the CPU (sherpa-onnx; nothing leaves the
robot). Her name wakes her. Since 2026-09-28 (``open_chat`` false) EVERY
sentence for her needs her name, except answers to her own questions; with
``open_chat`` true, once she is being talked to (a "conversation") you can
talk normally without it, until an ending or ``chat_timeout_s`` of silence:

    "Rosie, ..."                 anything with her name gets a reply and opens
                                 the conversation
    "Rosie, be quiet"            "ok", then every sound is muted (an ending)
    "Rosie, you can talk now"    unmuted, a happy trill
    "over and out"               her whole voice off: silent, and deaf to everything
                                 (her name too) but the code word to come back
    "rise and shine"             back on, with her hello. Both code words count
                                 only when said on their own, as the whole
                                 sentence, and "off" survives a reboot
    "thank you, Rosie" / "I have to go" / "bye" / "that's enough"
                                 endings: a word, and her name is needed again
    "stop" / "that's enough"     cuts her off mid-sentence, no complaint; she
                                 hears these even over her own voice (only these)
    "how are you?"               a story in her language; a spoken status
                                 report in English (health.py)
    "tell me a joke"             a laugh; "You're a joke" plus a laugh in English
    "what's the news / weather / stock market / going on in the world?"
                                 a short briefing (news.py), in about
                                 thirty-second pieces with "Want to hear more?"
    "Rosie, speak robot"         five minutes of beeps for her replies too;
                                 "speak English" ends it early
    "beep boop bleep"            beeping at her gets beeps back, that once
    "Rosie, in English"          the last thing she said, again, in words

Language (Steve, 2026-09-25, once Opus was her brain): a conversation is in
English by default - she answers in words, through the brain - and her system
sounds (hello at start-up, errors, the e-stop, "huh?" at a clap) stay robot.
Beeping at her, or "speak robot", gets her own voice back.

Anything else said to her in English goes to her brain (brain.py, Claude)
when it is awake; she starts with a lead-in chosen here - the topic echoed
back ("The Dodgers.") or a neutral opener ("Well,") - which the brain then
continues, so the pause for thinking sounds like the start of the answer.
In her own language she chatters and spends nothing; "Rosie, in English"
then asks the brain the same question.

Anyone may talk to her (Steve's choice, 2026-09-25), and she knows whose
voice it is once taught (speaker.py, 2026-09-26): "Rosie, learn my voice".
The first voice taught is the owner's. With him she is familiar and does
everything; with anyone else she is polite and brief, keeps his business and
her own status to herself, and mapping is his alone to start and stop. Once
someone has her ear she follows that voice; another voice must say her name.
Deaf while the motors run.
"""

import collections
import difflib
import glob
import json
import os
import queue
import random
import re
import subprocess
import sys
import threading
import time
import wave

import numpy as np

MODEL_DIR = os.path.expanduser('~/voice/models')
NAME = 'rosie'
# What the speech-to-text tends to make of her name -> rosie
NAME_VARIANTS = r"\b(rosie|rosy|rosey|rosi|rozy|rosee|rose e|rosie's|rosies|rosa|roszie|rozie)\b"
QUIET = ('be quiet', 'quiet', 'shut up', 'hush', 'silence', 'shush', 'stop talking', 'no more talking', 'zip it')
TALK = ('you can talk', 'talk now', 'you can speak', 'speak now', 'unmute', 'talk again', 'speak again',
        'you may talk', 'you may speak')
BYE = ('i have to go', 'i got to go', 'gotta go', 'got to go', 'bye', 'goodbye', 'good bye', 'see you', 'see ya',
       'talk to you later', 'great talking', 'nice talking', 'good talking', 'good night', 'catch you later',
       'i am leaving', "i'm leaving", 'i am going')
STOP = ("that's enough", 'that is enough', 'enough', 'stop', 'stop it', 'okay stop', 'stop please')
THANKS = ('thank you', 'thanks', 'thank you very much')
YES = ('yes', 'yeah', 'yep', 'yup', 'sure', 'more', 'go on', 'please', 'continue', 'keep going', 'okay', 'ok')
NO = ('no', 'nope', 'no thanks', 'no thank you', "that's all", 'that is all')
ENGLISH_OFF = ('speak robot', 'talk robot', 'speak rosie', 'your language', 'own language', 'robot language',
               'rosie language', 'stop speaking english', 'no more english', 'speak beeps', 'back to beeps',
               'back to normal', 'speak droid')
# Five minutes of English; the off phrases are checked first so "no more
# english" is not an on. Any other mention of English ("Rosie, in English",
# "say that in English") is a request to repeat the last thing in words.
ENGLISH_ON = ('speak english', 'talk english', 'switch to english', 'use english', 'english please',
              'english for now', 'english mode', 'go english', 'english now', 'english for a while')
ENGLISH_REPEAT = ('english',)
# Briefings (news.py); the specific kinds are checked before the general one.
NEWS = (
    ('world', ('world news', 'in the world', 'international news', 'global news', 'around the world')),
    ('us', ('u s news', 'us news', 'national news', 'american news', 'in america', 'in the country', 'in the states')),
    ('local', ('local news', 'around here', 'in town', 'local')),
    ('weather', ('weather', 'forecast', 'temperature', 'going to rain', 'raining', 'how hot', 'how cold', 'outside')),
    ('markets', ('stock market', 'stocks', 'the market', 'markets', 'the dow', 'nasdaq', 's and p', 'wall street')),
    ('all', ('news', 'headlines', "what's going on", 'what is going on', "what's happening", 'what is happening',
             "what's new", 'what is new')),
)

# Code words that switch her TALKING off and on without her name (Steve,
# 2026-09-25: her name kept waking her - said in the room, read out from a
# file). Off: she ignores everything said, even her name, asks her brain
# nothing and says nothing in English; her own sounds (hello, the clap's huh,
# alarms) carry on (Steve, 2026-09-26). They count only as the entire
# utterance, fillers aside.
VOICE_OFF = ('over and out', 'over and how', 'over an out')
# Said at normal speed "rise and shine" runs together into one word; what the
# recognizer made of it on 2026-09-27: "Rosentree", "Right in the chain",
# "The writing shine" (said slowly, with pauses, it hears it right).
VOICE_ON = ('rise and shine', 'rising shine', 'rise n shine', 'rise and shine rosie', 'rise in shine',
            'risenshine', 'risen shine', 'rise shine', 'rice and shine', 'writing shine', 'right and shine',
            'rising sign', 'rise and sign')
FILLERS = {'um', 'uh', 'erm', 'hmm', 'okay', 'ok', 'so', 'hey', 'alright', 'now', 'please', 'rosie'}
VOICE_OFF_FLAG = os.path.expanduser('~/voice/voice_off')


def code_word(text: str):
    """'off', 'on' or None - only when the code word is all that was said.
    Near misses count: the recognizer makes "rising shine" or "rise in shine"
    of it (2026-09-27, Steve said it three times to a deaf robot)."""
    words = [w for w in normalize(text).split() if w not in FILLERS]
    while words and words[0] in ('the', 'a', 'and'):
        words = words[1:]                   # "The writing shine"
    said = ' '.join(words)
    if not said:
        return None
    joined = said.replace(' ', '')
    for phrases, answer in ((VOICE_OFF, 'off'), (VOICE_ON, 'on')):
        for phrase in phrases:
            if (said == phrase or difflib.SequenceMatcher(None, said, phrase).ratio() >= 0.8
                    or difflib.SequenceMatcher(None, joined, phrase.replace(' ', '')).ratio() >= 0.8):
                return answer
    # a short utterance that ends in "shine" is her wake-up, however the start came out
    if len(words) <= 4 and words[-1] in ('shine', 'shines', 'shining', 'shined'):
        return 'on'
    return None


# Mapping on request (Steve, 2026-09-25): not at boot any more. Put her down at
# the parking spot, "Rosie, start mapping"; before picking her up, "Rosie,
# stop mapping" (saves the map). "napping" is what the recognizer often hears.
MAP_START = ('start mapping', 'begin mapping', 'start your map', 'start the map', 'start a map', 'start making a map',
             'turn on mapping', 'turn mapping on', 'mapping on', 'map the house', 'start napping', 'start map')
MAP_STOP = ('stop mapping', 'end mapping', 'finish mapping', 'done mapping', 'stop the map', 'stop your map',
            'turn off mapping', 'turn mapping off', 'mapping off', 'save the map', 'save your map', 'stop napping',
            'stop map')

# Watchdog mode for motion_watch (Steve, 2026-09-27): normally she only reacts
# to someone coming right at her; on watch she says "huh" and looks at every
# mover the lidar sees. Not remembered across a restart.
WATCH_ON = ('keep watch', 'watchdog mode', 'watch dog mode', 'guard mode', 'stand guard', 'on guard',
            'keep an eye out', 'guard the house', 'watch the house', 'start watching', 'be on watch')
WATCH_OFF = ('stand down', 'stop watching', 'at ease', 'watchdog off', 'watch dog off', 'guard off', 'off duty',
             'stop guarding', 'end watch')

# The camera's 3D map (nvblox) as a .ply (2026-09-27). normalize() drops digits,
# so "3D" arrives as "d".
SAVE_3D = ('save the d map', 'save d map', 'save the three d map', 'save three d map', 'save the d model',
           'save the three dimensional map', 'save your d map', 'save the d house')

HEALTH_WORDS = ('how are you', 'how you doing', 'you doing', 'how is it going', "how's it going", 'how are things',
                'how do you feel', 'how are you feeling', 'how is everything', "how's everything", 'you okay',
                'you all right', 'how are ya', 'how you feeling', 'feeling okay')
# The full spoken report, straight away (the banter above offers it).
REPORT_WORDS = ('status report', 'full status', 'full report', 'system status', 'systems report', 'diagnostics',
                'status')
HOW_AM_I = ('How am I?', 'Me?', 'How do I feel?')
JOKE_WORDS = ('joke', 'funny', 'make me laugh')
# "Rosie, think hard about ...": straight to Claude with full thinking (brain.py).
THINK_WORDS = ('think hard', 'think about it', 'think about this', 'think about that', 'think it over',
               'think it through', 'think this through', 'think that through', 'think carefully', 'really think',
               'take your time', 'think deeply', 'give it some thought', 'mull it over', 'think real hard',
               'use your brain', 'put your thinking cap on', 'thinking cap')
# Her own systems: to Steve these are answered from the live status, never by
# the chat model (it said YES to "is your lidar working?" without knowing).
SYSTEM_WORDS = ('lidar', 'laser', 'camera', 'cameras', 'sensor', 'sensors', 'imu', 'gyro', 'odometry', 'battery',
                'wifi', 'wi fi', 'guard', 'collision guard', 'systems', 'gpu', 'microphone', 'your mic', 'speaker',
                'working', 'online', 'offline', 'broken')
SPEND_WORDS = ('how much have you spent', 'spent today', 'how much money', 'what did that cost', 'your spending',
               'cost today', 'how much did you spend', 'how much are you costing')
# Voices (speaker.py). "Rosie, learn my voice" teaches her one; the first is the
# owner's. Steve, 2026-09-26: know when it's him, his wife or the TV, and act
# differently with him.
LEARN_VOICE = ('learn my voice', 'remember my voice', 'learn this voice', 'this is my voice', 'memorize my voice',
               'memorise my voice')
# Any learn/remember + voice counts: on 2026-09-27 "Can we do the Rosie learns
# my voice thing now?" went to the brain, which played along and pretended to
# learn his voice for three minutes.
LEARN_VOICE_RE = re.compile(r"\b(learn|learns|learning|remember|memori[sz]e|train on|record)\s+"
                            r"(?:my|this|our|me and my|his|her)?\s*voice\b")
FORGET_VOICE = ('forget my voice', 'forget this voice')
WHO_WORDS = ('who am i', 'who is this', "who's this", 'who is talking', "who's talking", 'who is speaking',
             "who's speaking", 'know my voice', 'do you know who i am', 'recognize me', 'recognise me',
             'recognize my voice', 'recognise my voice', 'who do you think i am')
NAME_PREFIX = re.compile(r"^(?:(?:hi|hello|hey|um|uh|oh|well|it's|its|it is|this is|i am|i'm|im|my name is|"
                         r"my name's|call me|the name is|the name's)\s+)+")
# words a mishearing leaves where a name should be; never a name
NOT_NAMES = {'on', 'off', 'and', 'the', 'a', 'an', 'to', 'of', 'in', 'is', 'it', 'at', 'or', 'so', 'but', 'yes',
             'yeah', 'no', 'nope', 'okay', 'ok', 'what', 'that', 'this', 'there', 'here', 'you', 'your', 'me', 'my',
             'i', 'we', 'he', 'she', 'they', 'just', 'like', 'name', 'voice', 'sorry', 'thanks', 'please'}
OWNER_ONLY = ('map_start', 'map_stop')
# Voice commands that make her DO something. Off until the new microphone array
# (Steve, 2026-09-27: "the voice commands have been basically unusable ... disable
# all voice commands that have the robot do anything"); she answers them with the
# "nope" sound so it is clear she heard. Parameter voice_actions turns them back on.
ROBOT_ACTIONS = ('map_start', 'map_stop', 'watch_on', 'watch_off', 'save_3d')          # hers to do only for the owner, once she knows his voice
GUEST_FINE = ("I'm doing fine, thanks for asking.", "Pretty good! Just keeping an eye on the house.",
              "All good here, thanks.")

# What she says in English when chatting (first matching subject wins, else
# she does not know - she is just a robot).
ENGLISH_REPLIES = (
    # Only what needs her own live data, plus Steve's joke: everything else
    # goes to the brain (small talk upstairs, real questions on to Opus).
    (REPORT_WORDS + HEALTH_WORDS, ["HEALTH"]),
    (SPEND_WORDS, ["SPEND"]),
    (('are you mapping', 'you mapping', 'are you napping', 'is mapping on', 'mapping status'), ["MAPPING"]),
    (JOKE_WORDS, ["You're a joke. [laugh]"]),        # [laugh]: the speak node adds the sound after the words
    (('what time is it', 'the time', "what's the time"), ["TIME"]),
    (('your battery', 'battery level', 'how much charge', 'battery at'), ["BATTERY"]),
)
ENGLISH_FILLERS = ["I don't know. I'm just a robot."]

# Lead-ins for the brain: never a stall, always the first words of the answer.
# Steve, 2026-09-25: no "hmm", no "good question", no "let me think".
LEAD_OPENERS = ('Well,', 'So,', 'Okay,', 'Right,', 'Honestly,', 'Oh,', "Let's see,", 'The way I see it,', 'Ah,', 'Now,')
LEAD_ECHO = re.compile(r"\b(?:about|of|like|love|hate|into|heard of|know about|think of|thoughts on|feel about)"
                       r"\s+(?:the |a |an |my |your |our )?([a-z]+(?: [a-z]+)?)\s*$")
LEAD_SKIP = {'it', 'that', 'this', 'you', 'me', 'them', 'him', 'her', 'us', 'there', 'here', 'now', 'today', 'rosie',
             'yourself', 'myself', 'something', 'anything', 'things', 'stuff', 'one', 'those', 'these'}
FETCH_LEAD = {'all': "The news. Here's what's going on.", 'world': '{ok} World news.', 'us': '{ok} U S news.',
              'local': '{ok} News from {city}.', 'weather': 'The weather in {city}.', 'markets': 'The markets.'}
# Her ways of saying okay (Steve, 2026-09-25: "sprinkle in some different words
# for okay" - and Ace Ventura's "Aaaalrighty then!", a sound made by speak.py).
OKAYS = ('Okay.', 'Alright.', 'Got it.', 'Sure thing.', 'You got it.', 'Righto.', 'Will do.', 'Roger that.',
         'Affirmative.')
ALRIGHTY_CHANCE = 0.25


def lead_in(text: str, last: str = None, rng=random) -> str:
    """The first words of her answer, chosen before the brain is asked: the
    topic echoed back when the sentence hands it over, else a neutral opener
    (never the same one twice running), else nothing about a third of the time."""
    t = normalize(text)
    m = LEAD_ECHO.search(t)
    if m:
        topic = m.group(1)
        if topic.split()[-1] not in LEAD_SKIP and topic.split()[0] not in LEAD_SKIP:
            return topic[0].upper() + topic[1:] + '.'
    if rng.random() < 0.33:
        return ''
    choices = [o for o in LEAD_OPENERS if o != last]
    return rng.choice(choices)


def known_subject(text: str) -> bool:
    """True when the English table has an answer of its own for this."""
    t = normalize(text)
    return any(_has(t, subjects) for subjects, _replies in ENGLISH_REPLIES)


def normalize(text: str) -> str:
    t = text.lower().replace('’', "'")
    t = re.sub(r"[^a-z' ]+", ' ', t)
    t = re.sub(NAME_VARIANTS, NAME, t)
    return re.sub(r'\s+', ' ', t).strip()


def _has(t: str, phrases) -> bool:
    return any(re.search(r'\b' + re.escape(p) + r'\b', t) for p in phrases)


def decide(text: str, mode: str):
    """What she does with what she heard -> (action, new mode).
    action: quiet | stop | thanks | talk | english | robot | repeat | bye | brief:<kind> | chat | None;
    mode: 'idle' | 'chat'."""
    t = normalize(text)
    addressed = re.search(rf'\b{NAME}\b', t) is not None
    # These two are specific enough to work even when her name got lost at
    # the start of the sentence (the model drops a soft first word now and then).
    if _has(t, ('speak robot', 'talk robot', 'speak rosie', 'speak beeps', 'speak droid')):
        return 'robot', mode
    if _has(t, ('speak english', 'talk english')):
        return 'english', 'chat'         # asking for English opens the conversation too
    if not (addressed or mode == 'chat'):
        return None, mode
    if _has(t, SAVE_3D):                # before MAP_STOP ("save the map")
        return 'save_3d', mode
    if _has(t, MAP_STOP):               # before STOP: "stop mapping" is not "stop"
        return 'map_stop', mode
    if _has(t, MAP_START):
        return 'map_start', mode
    if _has(t, WATCH_OFF):              # before STOP: "stop watching" is not "stop"
        return 'watch_off', mode
    if _has(t, WATCH_ON):
        return 'watch_on', mode
    if _has(t, FORGET_VOICE):
        return 'forget_voice', mode
    if _has(t, LEARN_VOICE) or LEARN_VOICE_RE.search(t):
        return 'learn_voice', 'chat'
    if _has(t, WHO_WORDS):
        return 'who', 'chat'
    if _has(t, QUIET) and addressed:     # a mute is sticky: only "Rosie, be quiet" (2026-09-25:
        return 'quiet', 'idle'            # "tell her to be quiet", said about someone else, muted her)
    if _has(t, STOP):
        return 'stop', 'idle' if 'enough' in t else mode
    if _has(t, THANKS):
        return 'thanks', 'idle'
    if _has(t, ENGLISH_OFF):
        return 'robot', mode
    if _has(t, ENGLISH_ON):
        return 'english', 'chat'
    for kind, words in NEWS:            # before "in english": "the weather, in English" is a briefing
        if _has(t, words):
            return f'brief:{kind}', 'chat'
    if _has(t, ENGLISH_REPEAT):
        return 'repeat', mode
    if _has(t, TALK):
        return 'talk', mode
    if _has(t, BYE):
        return 'bye', 'idle'
    return 'chat', 'chat'


def over_her_voice(text: str, own: str = ''):
    """The only things she takes from words spoken while she herself is
    talking (most of that is her own voice): a stop, or quiet - and not when
    the word is in what she is saying herself ("It's nice and quiet",
    a headline with "stop" in it): that is her, not you."""
    t = normalize(text)
    named = re.search(r'\b' + NAME + r'\b', t) is not None
    for action, phrases in (('map_stop', MAP_STOP), ('quiet', QUIET), ('stop', STOP)):
        if action == 'quiet' and not named:
            continue                      # muting needs her name, even over her own voice
        if any(re.search(r'\b' + re.escape(p) + r'\b', t) and not re.search(r'\b' + re.escape(p) + r'\b', own)
               for p in phrases):
            return action
    return None


def beyond_her_voice(text: str, own: str) -> str:
    """What someone said right after her, caught in the same stretch of sound
    as the end of her own sentence (2026-09-25: "...What's on your mind today?
    What's your full status report" was dropped whole as her echo). Her words
    are matched and cut off; what follows them is returned, or '' if nothing
    of a person's is left."""
    t = normalize(text).split()
    o = normalize(own).split()
    if len(t) < 2 or not o:
        return ''
    sm = difflib.SequenceMatcher(a=t, b=o, autojunk=False)
    blocks = [b for b in sm.get_matching_blocks() if b.size >= 2]
    if not blocks:
        return ''
    rest = t[max(b.a + b.size for b in blocks):]
    if len(rest) < 2:
        return ''
    if difflib.SequenceMatcher(a=rest, b=o, autojunk=False).ratio() > 0.6:
        return ''                      # more of her own words, heard badly
    return ' '.join(rest)


ROBOT_SOUNDS = re.compile(r"^(beep|beeps|boop|boops|bop|bops|bleep|bleeps|bloop|bloops|blip|blips|blorp|bip|bweep|"
                          r"beepity|boopity|bippity|boppity|doot|doo|dee|boo|bee|zzt|whirr|meep|moop|brr|ding|dong|"
                          r"beepboop|bleepbloop|weeoo|wee|woo|bzz|zap|zip)$")


def robot_talk(text: str) -> bool:
    """Someone beeping at her: mostly robot noises ("beep boop", "Rosie, bleep
    bloop blip") - she answers in kind."""
    words = [w for w in normalize(text).split() if w != NAME]
    hits = sum(1 for w in words if ROBOT_SOUNDS.match(w))
    return bool(words) and hits >= 1 and hits / len(words) >= 0.6


def english_reply(text: str, battery=None, rng=random, health=None) -> str:
    """Her side of a chat, in words. battery: (percent, present) or None;
    health: a jetnano_bringup.health.Health for "how are you?" (else a short answer)."""
    t = normalize(text)
    for subjects, replies in ENGLISH_REPLIES:
        if _has(t, subjects):
            reply = rng.choice(replies)
            if reply == 'HEALTH':
                if health is not None:
                    return health.report(rng)
                return rng.choice(["I'm doing great, thanks for asking!", "Pretty good! How are you?"])
            if reply == 'SPEND':
                try:
                    with open(os.path.expanduser('~/voice/brain_spend.json')) as f:
                        d = json.load(f)
                    if d.get('date') != time.strftime('%Y-%m-%d') or not d.get('exchanges'):
                        return "Nothing yet today. The brain upstairs is free."
                    cents = float(d.get('cents', 0.0))
                    money = f'{cents:.1f} cents' if cents < 100 else f'{cents / 100:.2f} dollars'
                    n = int(d.get('exchanges', 0))
                    return f"{money} today, over {n} question{'s' if n != 1 else ''} to my big brain."
                except (OSError, ValueError):
                    return "Nothing yet today. The brain upstairs is free."
            if reply == 'MAPPING':
                try:
                    on = subprocess.run(['systemctl', 'is-active', 'jetnano-slam'], capture_output=True, text=True,
                                        timeout=5).stdout.strip() in ('active', 'activating')
                except (OSError, subprocess.TimeoutExpired):
                    on = False
                return "Yes, I'm mapping." if on else "No, I'm not mapping right now."
            if reply == 'TIME':
                return time.strftime("It's %I:%M.").replace("'s 0", "'s ")
            if reply == 'BATTERY':
                if battery and battery[1] and battery[0] == battery[0]:
                    return f'My battery is at {int(round(battery[0] * 100))} percent.'
                return "I'm on wall power right now, so I'm fine."
            return reply
    return rng.choice(ENGLISH_FILLERS)


# ------------------------------------------------------------------ models --

def make_recognizer(model_dir: str, asr: str, threads: int):
    import sherpa_onnx
    if asr == 'whisper-tiny':
        m = os.path.join(model_dir, 'sherpa-onnx-whisper-tiny.en')
        return sherpa_onnx.OfflineRecognizer.from_whisper(
            encoder=f'{m}/tiny.en-encoder.int8.onnx', decoder=f'{m}/tiny.en-decoder.int8.onnx',
            tokens=f'{m}/tiny.en-tokens.txt', num_threads=threads, language='en', task='transcribe')
    if asr == 'moonshine-tiny':
        m = os.path.join(model_dir, 'sherpa-onnx-moonshine-tiny-en-int8')
        return sherpa_onnx.OfflineRecognizer.from_moonshine(
            preprocessor=f'{m}/preprocess.onnx', encoder=f'{m}/encode.int8.onnx',
            uncached_decoder=f'{m}/uncached_decode.int8.onnx', cached_decoder=f'{m}/cached_decode.int8.onnx',
            tokens=f'{m}/tokens.txt', num_threads=threads)
    raise ValueError(f'unknown asr "{asr}" (moonshine-tiny or whisper-tiny)')


def make_vad(model_dir: str, threshold: float, min_silence: float, min_speech: float, max_speech: float):
    import sherpa_onnx
    cfg = sherpa_onnx.VadModelConfig()
    cfg.silero_vad.model = os.path.join(model_dir, 'silero_vad.onnx')
    cfg.silero_vad.threshold = threshold
    cfg.silero_vad.min_silence_duration = min_silence
    cfg.silero_vad.min_speech_duration = min_speech
    cfg.silero_vad.max_speech_duration = max_speech
    cfg.sample_rate = 16000
    cfg.num_threads = 1
    return sherpa_onnx.VoiceActivityDetector(cfg, buffer_size_in_seconds=30), cfg.silero_vad.window_size


def transcribe(recognizer, samples: np.ndarray) -> str:
    stream = recognizer.create_stream()
    stream.accept_waveform(16000, samples)
    recognizer.decode_stream(stream)
    return stream.result.text.strip()


# -------------------------------------------------------------------- node --

def _node_main(args):
    import rclpy
    from geometry_msgs.msg import Twist
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Bool, Empty, Int16MultiArray, String, UInt32

    from jetnano_bringup import news, speaker, voice
    from jetnano_bringup.health import Health
    from jetnano_bringup.voice import ENGLISH

    class Listen(Node):

        def __init__(self):
            super().__init__('listen')
            self.declare_parameter('model_dir', MODEL_DIR)
            self.declare_parameter('asr', 'moonshine-tiny')          # or whisper-tiny
            self.declare_parameter('threads', 2)
            # The mic is tiny and inside the robot: a person a few feet away
            # reaches it far quieter than her own speaker does (2026-09-25,
            # Steve spoke and nothing triggered). Boost before the detector.
            # 15 -> 21 dB (Steve, 2026-09-25: "a hard time hearing us from across
            # the room"; his wife reached the mic at -43..-50 dBFS)
            self.declare_parameter('gain_db', 21.0)
            # the tail of her own sentence (and its echo) reaches the mic after
            # the speaking flag clears: 2026-09-25 she answered "I have lots",
            # "Voice section", "Of muscles" - her own last words, ~1 s late, -49 dBFS
            # 2026-09-28, the reSpeaker (echo-cancelled, her speaker on the same board):
            # her voice is gone 0.1-0.2 s after the flag clears, and the 1.5 s guard
            # threw away Steve's quick "My name is Steve." -> 0.6 s
            self.declare_parameter('echo_guard_s', 0.6)
            self.declare_parameter('highpass_hz', 100.0)
            self.declare_parameter('vad_threshold', 0.35)
            # a pause this long ends what you said. 0.5 -> 1.0 (Steve, 2026-09-28: "it doesn't
            # recognize that I'm talking, and then eventually talks over me"): half a second
            # of thinking mid-sentence was taken as the end, and the rest was lost under her
            self.declare_parameter('min_silence_s', 1.0)
            self.declare_parameter('min_speech_s', 0.3)
            self.declare_parameter('max_speech_s', 10.0)
            self.declare_parameter('chat_timeout_s', 45.0)          # silence that ends a conversation
            # False: only what has her name in it is for her - no open conversation after
            # "Rosie" (Steve, 2026-09-28, after "Word, the two in series." said to someone
            # else got "UNKNOWN" and "Okay." got "Anything on your mind?"). Answers to her
            # own questions (a name, "want to hear more?") still need no name.
            self.declare_parameter('open_chat', False)
            self.declare_parameter('more_timeout_s', 25.0)          # how long "want to hear more?" waits
            self.declare_parameter('still_after_s', 2.0)
            self.declare_parameter('chat_file', '/tmp/rosie_chat.wav')
            self.declare_parameter('location', '')                  # for local news and weather; '' = where the internet says
            self.declare_parameter('voice_model', speaker.DEFAULT_MODEL)   # '' or missing: no voice recognition
            self.declare_parameter('voices_dir', speaker.DEFAULT_DIR)
            self.declare_parameter('owner', 'Steve')
            self.declare_parameter('voice_actions', False)            # see ROBOT_ACTIONS
            self.declare_parameter('voice_match', speaker.MATCH)         # cosine: confidently that person
            self.declare_parameter('same_voice', speaker.SAME)           # cosine: the voice she is talking with

            p = lambda n: self.get_parameter(n).value  # noqa: E731
            model_dir = os.path.expanduser(str(p('model_dir')))
            t0 = time.monotonic()
            self.recognizer = make_recognizer(model_dir, str(p('asr')), int(p('threads')))
            self.min_silence = float(p('min_silence_s'))
            self.gain = 10.0 ** (float(p('gain_db')) / 20.0)
            self.echo_guard = float(p('echo_guard_s'))
            # Three quarters of the quiet room's "noise" at this mic is an
            # 11-15 Hz rumble (vibration or electrical, from inside the robot):
            # a 100 Hz high-pass takes the floor from -46 to -52.5 dBFS.
            self.hp, self.hp_state = None, None
            if float(p('highpass_hz')) > 0:
                try:
                    from scipy.signal import butter, sosfilt, sosfilt_zi
                    self.hp = butter(4, float(p('highpass_hz')), 'highpass', fs=16000, output='sos')
                    self.hp_state = sosfilt_zi(self.hp) * 0.0
                    self._sosfilt = sosfilt
                except ImportError:
                    self.get_logger().warning('no scipy: no high-pass filter')
            self.vad, self.window = make_vad(model_dir, float(p('vad_threshold')), self.min_silence,
                                             float(p('min_speech_s')), float(p('max_speech_s')))
            self.chat_timeout = float(p('chat_timeout_s'))
            self.open_chat = bool(p('open_chat'))
            self.more_timeout = float(p('more_timeout_s'))
            self.still_after = float(p('still_after_s'))
            self.chat_file = str(p('chat_file'))
            self.location = str(p('location'))
            self.owner = str(p('owner'))
            self.voices = None
            voice_model = os.path.expanduser(str(p('voice_model')))
            if voice_model and os.path.exists(voice_model):
                try:
                    self.voices = speaker.Voices(voice_model, os.path.expanduser(str(p('voices_dir'))), threads=1,
                                                 match=float(p('voice_match')), same=float(p('same_voice')))
                    self.get_logger().info(f'voices: knows {", ".join(self.voices.names()) or "nobody yet"}'
                                           f' (say "{NAME}, learn my voice")')
                except Exception as exc:      # noqa: BLE001 - she still hears without it
                    self.get_logger().warning(f'no voice recognition: {exc}')
            else:
                self.get_logger().warning(f'no voice recognition: {voice_model or "no model"} not found')
            self.chat_voice = None          # the voice print she is in conversation with
            self.enrol = None               # a voice lesson in progress

            self.text_pub = self.create_publisher(String, 'speech/text', 10)
            self.active_pub = self.create_publisher(Bool, 'speech/active', 10)
            self.state_pub = self.create_publisher(String, 'speech/state', 10)
            self.say_pub = self.create_publisher(String, 'say', 10)
            self.speak_pub = self.create_publisher(String, 'speak', 10)
            self.stop_pub = self.create_publisher(Empty, 'sound/stop', 10)
            self.ask_pub = self.create_publisher(String, 'brain/ask', 10)
            self.who_pub = self.create_publisher(String, 'speech/speaker', 10)
            self.watch_pub = self.create_publisher(String, 'motion/mode', 10)
            self.brain_ready = False
            self.last_opener = None
            self.create_subscription(Bool, 'brain/ready', self._on_brain_ready,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.mute_client = self.create_client(SetParameters, '/sounds/set_parameters')
            self.create_subscription(Int16MultiArray, 'sound/audio', self._on_audio, 10)
            self.create_subscription(Bool, 'sound/speaking', self._on_speaking, 10)
            self.create_subscription(Bool, 'sound/english', self._on_english,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(BatteryState, 'battery', self._on_battery, 10)
            self.create_subscription(String, 'sound/last', self._on_last, 10)
            # Everything she says in words passes over /speak (hers and the
            # sounds node's): kept, so her own words are never taken as yours.
            self.create_subscription(String, 'speak', self._on_speak, 10)
            self.own_text = collections.deque(maxlen=8)
            self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
            # Typed words count as heard: for tests, and for a page one day.
            # On their own thread, like spoken words: understanding can take
            # seconds (a status report samples topics) and must not hold the executor.
            self.create_subscription(
                String, 'speech/type',
                lambda m: threading.Thread(target=self._understand, args=(m.data, 2.0), daemon=True).start(), 10)
            self.create_timer(1.0, self._tick)
            self.health = Health(self)          # for "how are you?" in English

            self.queue = queue.Queue(maxsize=200)
            # A heartbeat from the listening thread itself, for the watchdog:
            # it stops if that thread hangs, even while the node looks alive.
            self.worked = 0.0
            self.beat_pub = self.create_publisher(UInt32, 'speech/heartbeat', 10)
            self.beats = 0
            self.create_timer(2.0, self._beat)
            self.mode = 'idle'
            self.muted = False
            self.english = False            # the sounds node's own switch (system sounds in words); not voice-set now
            self.voice_off = os.path.exists(VOICE_OFF_FLAG)   # "over and out" until "rise and shine"
            self.robot_until = 0.0          # "speak robot": her replies in beeps until then
            self.last_okay = None
            self.robot_for = 300.0
            self.battery = None
            self.last_sound = None          # what the sounds node played last (mood or file)
            self.last_question = None       # what her last chatter was an answer to
            self.last_action = None         # ... and what kind of question it was
            self.offer_until = 0.0          # "Want a full status report?" waits for yes or no until then
            self.terse = False              # Steve is talking: no chat, yes/no as sounds (2026-09-27)
            self.owner_at = -1e9            # when his voice was last recognised for sure
            self.offer_report = ''
            self.pending = []               # the rest of a briefing, after "want to hear more?"
            self.pending_until = 0.0
            self.active = False
            self.last_heard = 0.0
            self.last_cmd = 0.0
            self.speaking = False           # the sounds node is playing something
            self.speaking_ended = 0.0
            self._Parameter, self._ParameterType, self._ParameterValue, self._SetParameters = \
                Parameter, ParameterType, ParameterValue, SetParameters
            self._String, self._Bool, self._Empty = String, Bool, Empty
            threading.Thread(target=self._work, daemon=True, name='listen-asr').start()
            self.get_logger().info(f'{p("asr")} ready in {time.monotonic() - t0:.1f} s; say "{NAME}" to her')
            self._publish_state()

        # ------------------------------------------------------------ input --

        def _on_audio(self, msg) -> None:
            x = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
            if self.hp is not None:
                x, self.hp_state = self._sosfilt(self.hp, x, zi=self.hp_state)
                x = x.astype(np.float32)
            try:
                # a soft limit rather than a hard clip: someone close to her is
                # rounded off, not distorted, with this much gain
                self.queue.put_nowait(np.tanh(x * self.gain).astype(np.float32))
            except queue.Full:
                pass

        def _on_speaking(self, msg) -> None:
            now = time.monotonic()
            if self.speaking and not msg.data:
                self.speaking_ended = now
            self.speaking = msg.data
            self.last_heard = now           # her own talking keeps the conversation open

        def _on_english(self, msg) -> None:
            if msg.data != self.english:
                self.english = msg.data
                self.get_logger().info('speaking English' if msg.data else 'speaking Rosie')

        def _on_battery(self, msg) -> None:
            self.battery = (msg.percentage, msg.present)

        def _on_last(self, msg) -> None:
            self.last_sound = msg.data

        def _on_brain_ready(self, msg) -> None:
            if msg.data != self.brain_ready:
                self.brain_ready = msg.data
                self.get_logger().info('brain awake' if msg.data else 'brain asleep: her own words only')

        def _on_speak(self, msg) -> None:
            self.own_text.append(normalize(re.sub(r'\[\w+\]', '', msg.data)))

        def _on_cmd(self, msg) -> None:
            if msg.linear.x != 0.0 or msg.angular.z != 0.0:
                self.last_cmd = time.monotonic()

        def _work(self) -> None:
            buf = np.zeros(0, dtype=np.float32)
            history = np.zeros(0, dtype=np.float32)      # the last 2 s fed to the VAD
            fed = 0                                       # samples fed so far (VAD segment starts count in these)
            pre = int(0.3 * 16000)                        # a little before the VAD's start, for a soft first word
            while rclpy.ok():
                chunk = self.queue.get()
                self.worked = time.monotonic()
                if time.monotonic() - self.last_cmd < self.still_after:
                    buf = buf[:0]                         # motors running: deaf
                    continue
                buf = np.concatenate([buf, chunk])
                while len(buf) >= self.window:
                    window = buf[:self.window]
                    self.vad.accept_waveform(window)
                    buf = buf[self.window:]
                    fed += len(window)
                    history = np.concatenate([history, window])[-32000:]
                active = self.vad.is_speech_detected()
                if active != self.active:
                    self.active = active
                    flag = self._Bool()
                    flag.data = active
                    self.active_pub.publish(flag)
                while not self.vad.empty():
                    seg = self.vad.front
                    samples = np.array(seg.samples, dtype=np.float32)
                    self.vad.pop()
                    first = fed - len(history)            # absolute index of history[0]
                    i0, i1 = max(int(seg.start) - pre, first) - first, int(seg.start) - first
                    if 0 <= i0 < i1 <= len(history):
                        samples = np.concatenate([history[i0:i1], samples])
                    # did this start while she was talking? then it is mostly her
                    started = time.monotonic() - len(samples) / 16000.0 - self.min_silence
                    overlapped = self.speaking or started < self.speaking_ended + self.echo_guard
                    self._utterance(samples, overlapped)

        # ------------------------------------------------------------ words --

        def _utterance(self, samples: np.ndarray, overlapped: bool) -> None:
            seconds = len(samples) / 16000.0
            if self.voice_off:
                text = transcribe(self.recognizer, samples)
                if text and code_word(text) == 'on':
                    self._voice(True, text)
                elif text:
                    self.get_logger().info(f'talking is off; heard "{text[:60]}" and let it pass')
                return
            # level as it came off the mic, before the boost
            rms = float(np.sqrt(np.mean(samples * samples))) / self.gain if len(samples) else 0.0
            db = 20.0 * np.log10(max(rms, 1e-6))
            t0 = time.monotonic()
            text = transcribe(self.recognizer, samples)
            took = time.monotonic() - t0
            if not text:
                self.get_logger().info(f'speech with no words ({seconds:.1f} s at {db:.0f} dBFS)')
                return
            who = self._who(samples)
            if overlapped and code_word(text) == 'off':
                self._understand(text, seconds, took, db)
                return
            if overlapped and not self.speaking and who is not None and who.confident:
                # after she stopped, in the echo guard: her echo never sounds like a voice
                # she knows (2026-09-28: Steve's quick answer was dropped as her echo)
                self.get_logger().info(f'a known voice right after her own ({who.label}): taken')
                overlapped = False
                text = beyond_her_voice(text, ' '.join(list(self.own_text)[-3:])) or text
            if overlapped:
                action = over_her_voice(text, ' '.join(self.own_text))
                if action is None:
                    rest = beyond_her_voice(text, ' '.join(list(self.own_text)[-3:]))
                    if rest:
                        self.get_logger().info(f'heard after her own voice: "{rest[:80]}" ({db:.0f} dBFS)')
                        msg = self._String()
                        msg.data = rest
                        self.text_pub.publish(msg)
                        self._understand(rest, seconds, took, db, who)
                        return
                    self.get_logger().info(f'ignored over her own voice: "{text[:80]}" ({db:.0f} dBFS, '
                                           f'{who.label if who else "no voice print"})')
                if action == 'map_stop':
                    self.get_logger().info(f'heard "{text}" over her own voice -> stop mapping')
                    self._stop()
                    self._understand(text, seconds, took, db)
                elif action == 'stop':
                    self.get_logger().info(f'heard "{text}" over her own voice -> stop')
                    self._stop()
                elif action == 'quiet':
                    self.get_logger().info(f'heard "{text}" over her own voice -> quiet')
                    self._stop()
                    self._set_mute(True)
                    self.mode = 'idle'
                    self._publish_state()
                return
            msg = self._String()
            msg.data = text
            self.text_pub.publish(msg)
            self._understand(text, seconds, took, db, who)

        def _understand(self, text: str, seconds: float, took: float = 0.0, db: float = None, who=None) -> None:
            now = time.monotonic()
            word = code_word(text)
            if self.voice_off:
                if word == 'on':
                    self._voice(True, text)
                return
            if word == 'off':
                self._voice(False, text)
                return
            if self.enrol:                                           # she is learning a voice
                if now < self.enrol['until']:
                    if self._enrol_step(text, who):
                        return
                else:
                    self._enrol_end(timeout=True)
            if self.offer_until and now < self.offer_until:          # "Want a full status report?"
                t = normalize(text)
                self.offer_until = 0.0
                if _has(t, YES) and not _has(t, NO) and not _has(t, STOP):
                    self.get_logger().info(f'heard "{text}" -> the full report')
                    self.last_heard = now
                    self._speak(self.offer_report or english_reply('status report', self.battery, health=self.health))
                    return
                if _has(t, NO) or _has(t, STOP):
                    self.get_logger().info(f'heard "{text}" -> no report')
                    self.last_heard = now
                    self._okay()
                    return
            if self.pending and now < self.pending_until:       # "want to hear more?"
                t = normalize(text)
                if _has(t, YES) and not _has(t, NO) and not _has(t, STOP):
                    self.get_logger().info(f'heard "{text}" -> more')
                    self.last_heard = now
                    self._next_chunk()
                    return
                self.pending = []
                if _has(t, NO):
                    self.get_logger().info(f'heard "{text}" -> no more')
                    self._okay()
                    self.last_heard = now
                    return
            self.terse = self._owner_speaking(who)
            action, mode = decide(text, self.mode if self.open_chat else 'idle')
            level = f', {db:.0f} dBFS' if db is not None else ', typed'
            voice_note = f', {who.label}' if who is not None else ''
            self.get_logger().info(f'heard "{text}" ({seconds:.1f} s{level}{voice_note}, decoded in {took:.2f} s)'
                                   + (f' -> {action}' if action else '') + (' [terse]' if self.terse else ''))
            if action and not self._for_me(text, action, who):
                return
            if action in ROBOT_ACTIONS and not self.get_parameter('voice_actions').value:
                self.get_logger().info(f'{action}: voice commands that move or change her are off (voice_actions)')
                self._say('nope')
                return
            if mode != self.mode:
                self.mode = mode
                self._publish_state()
            if action:
                self.last_heard = now
            if action == 'quiet':
                self._stop()
                self._say('ok')
                self._set_mute(True, after=1.4)
            elif action == 'stop':
                self._stop()                              # no complaint; waiting for the next thing
            elif action == 'thanks':
                self._reply("You're welcome!", 'ok')          # to Steve: the yes sound
            elif action == 'talk':
                self._set_mute(False)
                self._say('happy')
            elif action == 'english':
                self.robot_until = 0.0                    # words again (they are the default)
                self._okay()
            elif action == 'robot':
                self.robot_until = time.monotonic() + self.robot_for
                self._say('boop')
            elif action == 'repeat':
                # once, in words; the language stays. A news question answered
                # with a story gets the real briefing now; an open question
                # answered with chatter goes to the brain.
                if self.last_sound == self.chat_file and (self.last_action or '').startswith('brief:'):
                    self._brief(self.last_action.split(':', 1)[1])
                elif (self.last_sound == self.chat_file and self.last_action == 'chat' and self.brain_ready
                      and self.last_question and not known_subject(self.last_question)):
                    self._ask_brain(self.last_question)
                else:
                    self._speak(self._last_in_english())
            elif action == 'learn_voice':
                self._learn_voice(who)
            elif action == 'forget_voice':
                self._forget_voice(who)
            elif action == 'who':
                self._who_am_i(who)
            elif action in ('map_start', 'map_stop'):
                self._mapping(action == 'map_start')
            elif action == 'save_3d':
                self._save_3d(announce=True)
            elif action in ('watch_on', 'watch_off'):
                m = String()
                m.data = 'watch' if action == 'watch_on' else 'approach'
                self.watch_pub.publish(m)
                self._reply("I'm on watch." if action == 'watch_on' else "Standing down.", 'ok')
            elif action == 'bye':
                self._say('bye')
            elif action and action.startswith('brief:'):
                # In her own language a news question gets a story (Steve,
                # 2026-09-25), unless English is on or asked for in the question.
                if self._words() or 'english' in normalize(text):
                    self._brief(action.split(':', 1)[1])
                else:
                    self.last_question, self.last_action = text, action
                    self._story()
            elif action == 'chat' and robot_talk(text):
                self.last_question, self.last_action = text, action
                n = int(min(12, max(3, round(2 + seconds * 2.2))))
                voice.write_wav(self.chat_file, voice.chat(n, rng=random))
                self._say(self.chat_file)
            elif action == 'chat' and self._words() and self.brain_ready and _has(normalize(text), THINK_WORDS):
                self._ask_brain(text, think=not self._guest(who), who=who)     # heavy thinking is the owner's
            elif action == 'chat' and self._words() and self._guest(who) and _has(normalize(text),
                                                                                    REPORT_WORDS + SPEND_WORDS):
                self._speak(f"That's between me and {self.owner}.")
            elif action == 'chat' and self._words() and self._guest(who) and _has(normalize(text), HEALTH_WORDS):
                self._speak(random.choice(GUEST_FINE))
            elif action == 'chat' and self._words():
                if _has(normalize(text), REPORT_WORDS):
                    if not self.terse:
                        self._okay('Checking.')                # the report samples for a couple of seconds
                    self._speak(english_reply('status report', self.battery, health=self.health))
                elif self.terse and (_has(normalize(text), HEALTH_WORDS) or _has(normalize(text), SYSTEM_WORDS)):
                    # Steve: what is wrong and the battery, nothing else
                    threading.Thread(target=lambda: self._speak(self.health.brief()), daemon=True).start()
                elif _has(normalize(text), HEALTH_WORDS):
                    self._how_am_i(text)
                elif known_subject(text) or not self.brain_ready:
                    self._speak(english_reply(text, self.battery, health=self.health))
                else:
                    self._ask_brain(text, who=who)
            elif action == 'chat':
                t = normalize(text)
                self.last_question, self.last_action = text, action
                if _has(t, JOKE_WORDS):
                    self._say('laugh')
                    return
                if _has(t, HEALTH_WORDS):       # "how are you?" gets the story: ~6 s (Steve: "it's funny")
                    self._story()
                else:
                    n = int(min(12, max(3, round(2 + seconds * 2.2))))
                    voice.write_wav(self.chat_file, voice.chat(n, rng=random))
                    self._say(self.chat_file)

        # ------------------------------------------------------------ voices --

        def _who(self, samples: np.ndarray):
            """The voice print of what was just heard, against the voices she knows."""
            if self.voices is None:
                return None
            try:
                w = self.voices.who(samples)
            except Exception as exc:      # noqa: BLE001 - never lose the words over the voice
                self.get_logger().warning(f'voice print failed: {exc}')
                return None
            if w is not None:
                msg = self._String()
                msg.data = json.dumps({'name': w.name if w.confident else None, 'nearest': w.name,
                                       'score': round(w.score, 3), 'percent': w.percent})
                self.who_pub.publish(msg)
            return w

        def _guest(self, who) -> bool:
            """Not the owner, by voice - only once she knows the owner's voice. Typed words are his."""
            return (self.voices is not None and self.voices.has(self.owner) and who is not None
                    and not who.is_owner(self.owner))

        def _describe(self, who) -> str:
            if self.voices is None or not self.voices.names():
                return f'{self.owner} or someone else in the room (you cannot tell voices apart yet)'
            if who is None:
                return f'{self.owner}, your owner (typed)'
            return who.describe(self.owner)

        def _for_me(self, text: str, action: str, who) -> bool:
            """Is this for her, and may this voice ask it? Whoever says her name has
            her ear; in a conversation another voice is ignored until it does.
            Mapping is the owner's alone once she knows his voice."""
            if who is not None and who.emb is not None:
                addressed = re.search(rf'\b{NAME}\b', normalize(text)) is not None
                if addressed or self.mode != 'chat' or self.chat_voice is None:
                    self.chat_voice = who.emb
                elif who.seconds >= 2.0 and self.voices.cosine(who.emb, self.chat_voice) < self.voices.same:
                    # (a word or two is too short to judge: Steve's own "Hello" was
                    # taken for another voice with nobody enrolled yet, 2026-09-27)
                    self.get_logger().info(f'another voice ({who.label}), not talking to me: "{text[:60]}"')
                    return False
            if action in OWNER_ONLY and self._guest(who):
                self.get_logger().info(f'{action} asked by {who.label}: only {self.owner} may')
                self._reply(f'Only {self.owner} can ask me that.', 'hm')
                return False
            return True

        def _owner_speaking(self, who) -> bool:
            """Is it Steve? Typed words are his. Once she knows his voice: a sure
            match, or a short or unsure one that sounds nearest him within a
            minute of a sure one (a "yes" is too short to be sure of)."""
            if who is None:
                return True
            if self.voices is None or not self.voices.has(self.owner):
                return False
            now = time.monotonic()
            if who.is_owner(self.owner):
                self.owner_at = now
                return True
            return (not who.confident and who.name is not None and who.name.lower() == self.owner.lower()
                    and who.score >= self.voices.same and now - self.owner_at < 60.0)

        def _learn_voice(self, who) -> None:
            if self.voices is None:
                self._speak("I can't learn voices right now.")
                return
            if who is None or who.emb is None:
                self._speak('Say that again, a little longer, and I will learn it.')
                return
            until = time.monotonic() + 60.0
            audio = [who.audio]
            if who.confident:
                self.enrol = {'name': who.name, 'embs': [who.emb], 'first': who.emb, 'until': until, 'target': 4,
                              'audio': audio}
                self._speak(f'I know your voice, {who.name}. Say a few more sentences and I will know it even better.')
            elif not self.voices.has(self.owner):
                self.enrol = {'name': self.owner, 'embs': [who.emb], 'first': who.emb, 'until': until, 'target': 6,
                              'audio': audio}
                self._speak(f'Okay, {self.owner}. Say a few sentences to me, anything at all, '
                            'and I will remember your voice.')
            else:
                self.enrol = {'name': None, 'embs': [who.emb], 'first': who.emb, 'until': until, 'target': 6,
                              'audio': audio}
                self._speak("Happy to. What's your name?")

        def _enrol_step(self, text: str, who) -> bool:
            """One more utterance while she learns a voice. True: it was part of the lesson."""
            e = self.enrol
            t = normalize(text)
            if _has(t, STOP) or _has(t, BYE):
                self._enrol_end()
                return True
            if who is None or who.emb is None:
                return True                                  # too short to use
            # like any sample so far, not just the first: that one can be short and poor
            # (2026-09-28: Steve's own next sentence scored "a different one" against it)
            if max(self.voices.cosine(who.emb, x) for x in e['embs']) < self.voices.same:
                self.get_logger().info(f'learning a voice: a different one ({who.label}), ignored: "{text[:60]}"')
                return True
            if e['name'] is None:
                name = NAME_PREFIX.sub('', t).strip().split(' ')[0] if t else ''
                # 2026-09-28 she took "On" for Steve's name and learned him as "On"
                if len(name) < 2 or name == NAME or name in NOT_NAMES:
                    self.get_logger().info(f'learning a voice: no name in "{text[:60]}"')
                    self._speak("Sorry, I didn't catch your name. Just your name, please.")
                    return True
                e['name'] = self.voices.canonical(name[0].upper() + name[1:])
                self.get_logger().info(f'learning a voice: the name is {e["name"]}, from "{text[:60]}"')
                e['embs'].append(who.emb)
                e['audio'].append(who.audio)
                self._speak(f"Nice to meet you, {e['name']}. Say a few more sentences to me.")
                return True
            e['embs'].append(who.emb)
            e['audio'].append(who.audio)
            e['until'] = time.monotonic() + 60.0
            if len(e['embs']) >= e['target']:
                self._enrol_end()
            elif len(e['embs']) == max(2, e['target'] // 2):
                self._speak('Keep going, a couple more.')
            return True

        def _enrol_end(self, timeout: bool = False) -> None:
            e, self.enrol = self.enrol, None
            if not e or not e['name'] or len(e['embs']) < 2:
                self._speak('Never mind about the voice, then.')
                return
            for emb in e['embs']:
                self.voices.add(e['name'], emb)
            try:
                self.voices.save_samples(e['name'], e.get('audio', []))     # for re-scoring when tuning
            except OSError as exc:
                self.get_logger().warning(f'could not keep the voice samples: {exc}')
            self.chat_voice = e['embs'][-1]
            self.get_logger().info(f'voice learned: {e["name"]}, {len(e["embs"])} samples; '
                                   f'knows {", ".join(self.voices.names())}')
            self._speak(f"Got it, {e['name']}. I'll know your voice now.")

        def _forget_voice(self, who) -> None:
            if self.voices is None or who is None or not who.confident:
                self._speak("I don't know your voice anyway.")
                return
            self.voices.forget(who.name)
            self._speak(f"Okay, I've forgotten your voice, {who.name}.")

        def _who_am_i(self, who) -> None:
            if self.voices is None:
                self._speak("I can't tell voices apart right now.")
            elif not self.voices.names():
                self._speak(f"I don't know any voices yet. Say, {NAME}, learn my voice.")
            elif who is None or who.emb is None:
                self._speak('Say a little more and I will tell you.')
            elif who.confident and self.terse:
                self._speak(f'{who.name}. {who.percent} percent.')
            elif who.confident:
                self._speak(f"That's you, {who.name}. I'm {who.percent} percent sure.")
            elif who.name and who.score >= 0.25:
                self._speak(f"You sound a bit like {who.name}, but I'm only {who.percent} percent sure.")
            else:
                self._speak("I don't know your voice.")

        def _voice(self, on: bool, text: str) -> None:
            """The code words: her talking off (sleepy, then deaf to all but "rise
            and shine" and no English, brain or briefings; her sounds carry on)
            or back on (hello, and unmuted if she had been told to be quiet)."""
            self.get_logger().info(f'heard "{text}" -> voice {"ON" if on else "OFF"}')
            if on:
                self.voice_off = False
                try:
                    os.remove(VOICE_OFF_FLAG)
                except OSError:
                    pass
                self._set_mute(False)
                self._say('hello')
            else:
                self._stop()
                self.pending, self.offer_until = [], 0.0
                self.mode = 'idle'
                self._say('sleepy')
                self.voice_off = True                 # her sounds stay on: only the talking stops
                os.makedirs(os.path.dirname(VOICE_OFF_FLAG), exist_ok=True)
                with open(VOICE_OFF_FLAG, 'w') as f:
                    f.write(time.strftime('%Y-%m-%d %H:%M:%S') + '\n')
            self._publish_state()

        def _okay(self, then: str = '') -> None:
            """Okay, in one of her ways - now and then "Aaaalrighty then!" - and
            whatever follows it. To Steve: the yes sound, nothing else."""
            if self.terse:
                self._say('yes')
                return
            if random.random() < ALRIGHTY_CHANCE and glob.glob(os.path.expanduser('~/sounds/alrighty[0-9]*.wav')):
                self._say('alrighty')
                if then:
                    self._speak(then)
                return
            word = random.choice([o for o in OKAYS if o != self.last_okay])
            self.last_okay = word
            self._speak((word + ' ' + then).strip())

        def _words(self) -> bool:
            """Replies in English, unless "speak robot" is in force."""
            return time.monotonic() >= self.robot_until

        def _reply(self, words: str, mood: str) -> None:
            """Words by default, her own sound in a robot spell. To Steve a good
            outcome is the yes sound; anything else keeps its (short) words."""
            if self.terse and mood in ('ok', 'happy'):
                self._say('yes')
            elif self._words():
                self._speak(words)
            else:
                self._say(mood)

        def _save_3d(self, announce: bool) -> None:
            """nvblox's 3D map to ~/maps3d/*.ply (save_3d_map.sh), in the background."""
            def run():
                try:
                    from ament_index_python.packages import get_package_prefix
                    script = os.path.join(get_package_prefix('jetnano_bringup'), 'lib', 'jetnano_bringup',
                                          'save_3d_map.sh')
                    r = subprocess.run(['bash', script], capture_output=True, text=True, timeout=120)
                except Exception as exc:      # noqa: BLE001
                    r = None
                    self.get_logger().warning(f'3D map not saved: {exc}')
                ok = r is not None and r.returncode == 0
                if ok:
                    self.get_logger().info(f'3D map saved: {r.stdout.strip()}')
                elif r is not None:
                    self.get_logger().warning(f'3D map not saved: {r.stderr.strip()[:160]}')
                if announce:
                    self._reply('Saved my 3D map.' if ok else "I couldn't save my 3D map.", 'ok' if ok else 'no')
            threading.Thread(target=run, daemon=True, name='listen-save3d').start()

        def _mapping(self, start: bool) -> None:
            """Start or stop jetnano-slam. Stopping saves the map (slam_boot.sh
            saves before slam_toolbox quits); this checks the file really changed."""
            def run():
                def active():
                    return subprocess.run(['systemctl', 'is-active', 'jetnano-slam'], capture_output=True,
                                          text=True, timeout=10).stdout.strip() in ('active', 'activating')
                graph = os.path.expanduser('~/maps/home.posegraph')
                try:
                    if start and active():
                        self._reply("I'm already mapping.", 'ok')
                        return
                    if not start and not active():
                        self._reply("I'm not mapping right now.", 'ok')
                        return
                    if not self._words():
                        self._say('ok')
                    elif start:
                        self._okay("Starting my map. I'm counting on being at my parking spot.")
                    else:
                        self._okay('Saving my map and stopping.')
                    before = os.path.getmtime(graph) if os.path.exists(graph) else 0.0
                    r = subprocess.run(['sudo', '-n', 'systemctl', 'start' if start else 'stop', 'jetnano-slam'],
                                       capture_output=True, text=True, timeout=150)
                    if r.returncode != 0:
                        self.get_logger().warning(f'jetnano-slam {"start" if start else "stop"}: {r.stderr.strip()[:160]}')
                        self._reply(f"I couldn't {'start' if start else 'stop'} my map.", 'no')
                        return
                    if start:
                        self.get_logger().info('mapping started')
                        self._reply("I'm mapping now.", 'happy')
                    else:
                        after = os.path.getmtime(graph) if os.path.exists(graph) else 0.0
                        saved = after > before
                        self.get_logger().info(f'mapping stopped, map {"saved" if saved else "NOT saved"}')
                        self._save_3d(announce=False)          # the camera's 3D model too
                        self._reply('Map saved. You can pick me up now.' if saved else
                                    "I stopped, but the map didn't save.", 'ok' if saved else 'sad')
                except (OSError, subprocess.TimeoutExpired) as exc:
                    self.get_logger().warning(f'mapping: {exc}')
                    self._reply("Something went wrong with my map.", 'no')
            threading.Thread(target=run, daemon=True, name='listen-mapping').start()

        def _story(self) -> None:
            voice.write_wav(self.chat_file, voice.story(random.randrange(4, 6), rng=random))
            self._say(self.chat_file)

        def _beat(self) -> None:
            if time.monotonic() - self.worked < 3.0:
                self.beats += 1
                msg = UInt32()
                msg.data = self.beats
                self.beat_pub.publish(msg)

        def _tick(self) -> None:
            if self.mode == 'chat' and time.monotonic() - self.last_heard > self.chat_timeout:
                self.mode = 'idle'
                self.pending = []
                self.get_logger().info('the conversation went quiet')
                self._publish_state()

        # --------------------------------------------------------- briefing --

        def _how_am_i(self, text: str) -> None:
            """'How are you?': a line or two of banter from her real status (the
            model upstairs, free), then 'Want a full status report?'."""
            opener = random.choice(HOW_AM_I)
            self._speak(opener)

            def run():
                report = self.health.report()            # samples the sensors for a couple of seconds
                self.offer_report = report
                self.offer_until = time.monotonic() + 45.0
                if self.brain_ready:
                    msg = self._String()
                    msg.data = json.dumps({'text': text, 'lead_in': opener, 'health': report,
                                           'then_say': 'Want a full status report?'})
                    self.ask_pub.publish(msg)
                    self.get_logger().info('asks the brain for banter about how she is')
                else:
                    self._speak(report.split('\n')[0] + '\nWant a full status report?')
            threading.Thread(target=run, daemon=True, name='listen-howami').start()

        def _ask_brain(self, text: str, think: bool = False, who=None) -> None:
            # "think hard": the brain says "Give me a moment..." itself, no lead-in;
            # Steve gets no lead-in at all
            lead = '' if think or self.terse else lead_in(text, self.last_opener)
            if lead:
                self._speak(lead)
                self.last_opener = lead if lead in LEAD_OPENERS else self.last_opener
            self.get_logger().info(f'asks the brain "{text[:80]}"' + (f' after "{lead}"' if lead else '')
                                   + (' to think hard' if think else ''))
            msg = self._String()
            msg.data = json.dumps({'text': text, 'lead_in': lead, 'think': think, 'speaker': self._describe(who),
                                   'terse': self.terse})
            self.ask_pub.publish(msg)

        def _brief(self, kind: str) -> None:
            city = self.location.partition(',')[0].strip()
            self.brief_terse = self.terse
            if not self.terse:                  # Steve gets the news, not "Okay, here's the news"
                lead = FETCH_LEAD[kind].replace('{ok}', random.choice(OKAYS))
                if '{city}' in lead:
                    lead = (lead.format(city=city) if city
                            else lead.replace(' in {city}', ' here').replace(' from {city}', ' from around here'))
                self._speak(lead)

            def fetch():
                try:
                    sentences = news.briefing(kind, self.location)
                except Exception as exc:      # noqa: BLE001
                    self.get_logger().warning(f'briefing failed: {exc}')
                    self._speak("I couldn't reach the news right now.")
                    return
                self.pending = news.chunks(sentences)
                self.get_logger().info(f'{kind} briefing: {len(sentences)} sentences in {len(self.pending)} piece(s)')
                self._next_chunk()

            threading.Thread(target=fetch, daemon=True, name='listen-news').start()

        def _next_chunk(self) -> None:
            if not self.pending:
                return
            lines = self.pending.pop(0)
            more = bool(self.pending)
            words = sum(len(s.split()) for s in lines)
            ask = more and not getattr(self, 'brief_terse', False)   # Steve says "more" if he wants it
            self._speak('\n'.join(lines) + ('\nWant to hear more?' if ask else ''))
            self.pending_until = time.monotonic() + words / 2.5 + 6.0 + self.more_timeout
            self.last_heard = time.monotonic()

        # ----------------------------------------------------------- output --

        def _last_in_english(self) -> str:
            """The last thing she said, in words."""
            last = self.last_sound
            if last is None:
                return "I haven't said anything yet."
            if last in (self.chat_file, 'laugh') and self.last_question:
                return english_reply(self.last_question, self.battery, health=self.health)
            if last in ENGLISH:
                return random.choice(ENGLISH[last])
            if last.endswith('.wav'):
                return 'That was already in English.'
            return "I don't know how to say that in English."

        def _say(self, what: str) -> None:
            msg = self._String()
            msg.data = what
            self.say_pub.publish(msg)

        def _speak(self, words: str) -> None:
            if self.speak_pub.get_subscription_count() == 0:
                self.get_logger().warning('no speak node: cannot say it in English')
                self._say('hm')
                return
            self.get_logger().info(f'says "{words.replace(chr(10), " | ")[:500]}"')
            msg = self._String()
            msg.data = words
            self.speak_pub.publish(msg)

        def _stop(self) -> None:
            self.pending = []
            self.stop_pub.publish(self._Empty())

        def _set_param(self, name: str, on: bool) -> None:
            if not self.mute_client.service_is_ready():
                self.get_logger().warning(f'sounds node not there: cannot set {name}')
                return
            req = self._SetParameters.Request()
            prm = self._Parameter()
            prm.name = name
            prm.value = self._ParameterValue(type=self._ParameterType.PARAMETER_BOOL, bool_value=on)
            req.parameters = [prm]
            self.mute_client.call_async(req)

        def _set_mute(self, on: bool, after: float = 0.0) -> None:
            def do():
                self.muted = on
                self._publish_state()
                self._set_param('mute', on)
            if after > 0:
                t = threading.Timer(after, do)
                t.daemon = True
                t.start()
            else:
                do()

        def _publish_state(self) -> None:
            msg = self._String()
            msg.data = 'voice off' if self.voice_off else 'muted' if self.muted else self.mode
            self.state_pub.publish(msg)

    rclpy.init(args=args)
    node = Listen()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ---------------------------------------------------------- files (a test) --

def _files_main(argv):
    asr = argv[argv.index('--asr') + 1] if '--asr' in argv else 'moonshine-tiny'
    paths = [a for a in argv[argv.index('--file') + 1:] if not a.startswith('--') and a != asr]
    t0 = time.monotonic()
    rec = make_recognizer(os.environ.get('MODEL_DIR', MODEL_DIR), asr, 2)
    print(f'{asr}: loaded in {time.monotonic() - t0:.1f} s')
    for path in paths:
        with wave.open(path) as w:
            assert w.getframerate() == 16000 and w.getnchannels() == 1, f'{path}: need 16 kHz mono'
            x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32) / 32768.0
        t0 = time.monotonic()
        text = transcribe(rec, x)
        took = time.monotonic() - t0
        idle, chat = decide(text, 'idle'), decide(text, 'chat')
        print(f'{os.path.basename(path):16s} {len(x) / 16000:4.1f} s  {took:5.2f} s  "{text}"'
              f'  idle->{idle[0]}  chat->{chat[0]}'
              + (f'  english: "{english_reply(text)}"' if chat[0] == 'chat' else ''))
    return 0


def main(args=None):
    argv = sys.argv[1:]
    if '--file' in argv:
        sys.exit(_files_main(argv))
    if '--news' in argv:
        from jetnano_bringup import news
        sys.exit(news.main(argv[argv.index('--news') + 1:]))
    _node_main(args)


if __name__ == '__main__':
    main()
