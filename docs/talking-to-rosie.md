# Talking to Rosie

Say her name first. After that she listens to everything for about 45
seconds, so you can talk normally; the conversation ends with a thank-you,
a goodbye, "that's enough", or silence. Anyone can talk to her, and she can
learn whose voice is whose (below): once someone has her ear she follows that
voice, and another voice has to say her name to cut in.

She answers in English. Her system sounds - starting up, errors, the
emergency stop, the "huh?" when she hears a clap - are her own beeps.

## Everyday

| Say | She does |
|---|---|
| "Rosie, how are you?" | a line or two about how she really is (sensors, battery, heat, the room), then "Want a full status report?" - yes or no |
| "Rosie, status report" | the full report straight away |
| "Rosie, tell me a joke" | "You're a joke." and a laugh |
| "Rosie, what's the weather?" | the weather here |
| "What's the news?" / "world news" / "local news" / "how's the stock market?" | a short briefing, with "Want to hear more?" every half minute |
| "What time is it?" / "How's your battery?" | the answer |
| "How much have you spent today?" | what her Claude questions have cost today |
| anything else | an answer from her brain (below) |
| "Thank you, Rosie" / "I have to go" / "that's enough" | the conversation ends |

## Stopping her

| Say | She does |
|---|---|
| "Stop" or "that's enough" | stops talking at once, even mid-sentence |
| "Rosie, be quiet" | goes silent until told otherwise - her name is needed |
| "Rosie, you can talk now" | talks again (a happy trill) |

## Switching her voice off completely

For when her name keeps coming up in the room. These code words work only
when said on their own - "Over and out." - not inside a longer sentence.

| Say | She does |
|---|---|
| "Over and out" | a sleepy sound, then silence: she ignores everything, even her name, and makes no sounds at all |
| "Rise and shine" | her voice back on, with a hello |

Off stays off through a restart or a reboot until "rise and shine".

## Thinking

Small talk is answered by the model on the PC upstairs, free and quick.
Real questions - facts, numbers, science, history, how things work, advice -
go to Claude (Opus 5.5), a fraction of a cent each; the first word you hear
("Well," or the topic said back to you) covers the moment it takes.

| Say | She does |
|---|---|
| "Rosie, think hard about ..." (or "take your time", "think it through") | "Give me a moment..." and a considered answer, a few seconds |
| "Rosie, in English" | says her last beeping reply again, in words |

## Her own language

| Say | She does |
|---|---|
| "Beep boop!" | beeps back (never costs anything) |
| "Rosie, speak robot" | beeps instead of words for five minutes |
| "Rosie, speak English" | words again |

## Mapping

Mapping does not start by itself. Put her at the parking spot first - it is
where every map begins.

| Say | She does |
|---|---|
| "Rosie, start mapping" | starts mapping (about 30 seconds to begin) |
| "Rosie, stop mapping" | saves the map and stops - do this before picking her up |
| "Rosie, are you mapping?" | yes or no |

## If she seems to ignore you

- She may be muted: "Rosie, you can talk now".
- The microphone is small: from across the room, speak up a little.
- She ignores anything that starts while she is talking, except "stop" and
  "Rosie, be quiet" - wait for her to finish, or say "stop".
- The driving page (http://192.168.1.7:8081/) shows a warning line when the
  watchdog is fixing something.

## Whose voice

She takes a voice print of everything she hears and compares it with the
voices she has been taught. It is not security - a recording would pass - it
is knowing who is in the room. The first voice she learns is Steve's.

| Say | She does |
|---|---|
| "Rosie, learn my voice" | asks for a few sentences, then remembers the voice (the first voice learned is the owner's; anyone after that is asked their name) |
| "Rosie, who am I?" / "do you know my voice?" | "That's you, Steve. I'm 80 percent sure." |
| "Rosie, forget my voice" | forgets it |

With Steve she is familiar and does everything. With anyone else she is
polite and brief, keeps his business (the status report, her spending) to
herself, "think hard" is answered without the slow deep thinking, and
mapping is his alone to start and stop: "Only Steve can ask me that."

Tuning: `ros2 run jetnano_bringup voices` lists who she knows;
`ros2 topic echo /speech/speaker` shows the score of each thing she hears.
The thresholds are listen's `voice_match` (confidently that person) and
`same_voice` (the voice she is talking with); the model is WeSpeaker CAM++
via sherpa-onnx in `~/voice/models/speaker/`.
