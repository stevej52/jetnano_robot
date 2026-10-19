# Rosie's conversation: what makes it snappy

Tuned 2026-10-18 for Grandma Linda (Diane's mother, 80, loves to talk), first talked with by Diane the
same night. Steve: "Much better voice now. Very close to perfect." Keep these settings unless a test
shows something better.

## The numbers

| | Before (10-15, 10-18 morning) | After (10-18 night, 24 turns with Diane) |
|---|---|---|
| Speech-to-text left after the speaker stops | median 1.3 s, 6 s after 30 s of talk, 16 s after 87 s | median 0.43 s, max 1.2 s |
| Brain's first words | median 3.0 s (10-15), two 30 s stalls | median 0.3 s |
| Brain's whole reply | median 4.7 s (10-15) | median 1.6 s |
| Prompt the local model reads per reply | the whole conversation, every minute | ~120 new tokens, ~1,280 cached |
| Voice server off (H2-Host) | +3.2-3.6 s on half the replies | no wait at all |

## What does it

### Hearing (`listen.py`)
- **Speech-to-text while they talk** (`early_asr` true, `early_chunk_s` 4.0): every 4 s of ongoing
  speech is cut at its quietest 20 ms in the last second (between words) and decoded in the
  background; when they stop, only the rest is decoded. All decodes go through one thread
  (`asr_pool`). The first piece starts 0.8 s before the VAD noticed speech, never before her own
  voice ended + `echo_guard`. Cost: about 3 word slips per 30 s at the cuts.
- Turn-taking, unchanged: `min_silence_s` 0.4 (a pause is a check), Smart Turn `turn_done_p` 0.5 /
  `turn_sure_p` 0.8, holds `hold_maybe_s` 2.0, `hold_unfinished_s` 20.0, `hold_name_s` 2.5.
- Speech recogniser: Parakeet TDT 0.6B v2 int8, 2 threads (`asr` parakeet).
- `JOKE_WORDS` only real requests ("tell me a joke", "say something funny"), not any "funny".
- Voices: `voice_relations` "Linda=Diane's mother, Steve's mother-in-law;Diane=Steve's wife". A
  woman's voice the print takes for Steve's is never added to his print in a voice lesson.
- **Who is this?** (`ask_who` true, Steve 2026-10-19: Linda will never do a voice lesson). A voice
  that is not Steve's and that she cannot name gets "Is this Diane?" (a woman's voice) or "Who am I
  talking to?"; what they said is kept and answered once she knows. "Yes" = Diane; "No, this is
  Linda" = Linda; a bare "No" gets "Oh, sorry! Who am I talking to?". Then "Hi, Linda!" and her first
  words are answered as Linda. That voice is that person for the visit (`session_voice_s` 3600 after
  it last spoke), matched at `session_match` 0.5 or better, with more prints added as she talks; a
  close call between two people stays with whoever just spoke; a reply too short to print ("yes") is
  whoever spoke in the last minute. No name given: that voice is not asked again for 10 minutes.
  "Rosie, this is Linda" does the same without being asked. In Diane mode a named (or not yet asked)
  voice is in the conversation without saying "Rosie".
- Test voices: two Kokoro voices are NOT a fair test of telling people apart (CAM++ scores them
  0.27-0.90 against each other); Steve's real clips score 0.62-0.84 among themselves and 0.2-0.34
  against them. The real Diane-vs-Linda separation is still to be seen live.

### Brain (`brain.py`)
- **Cache-friendly prompt for the local model**: the system prompt holds only what does not change
  (PERSONA + HAND_OVER + the person's block TERSE / FUN / LINDA); the time, battery, who is talking
  and the lead-in go at the END of the newest message (`LOCAL_NOTE`, `LOCAL_LEAD_IN` without
  examples). Before, the time (changing every minute) sat ahead of the conversation, so the server
  re-read all of it. The log shows "read N new + M cached".
- **Read-ahead** (`warm_up` true): when `speech/active` goes on, the conversation so far is sent with
  one token back (at most every 15 s), so the server holds it ready. Log: "read ahead: ...".
- **Fast hand-over** (`local_timeout_s` 30 -> 6): a local model that has not started in 6 s hands the
  answer to Claude (claude-opus-5-5, quick effort) instead of "I can't reach my brain right now".
- No doubled lead-in: `_unrepeat` strips the whole lead-in ("The way I see it,") off the answer's start.
- Laughs: "Ha!" / "Haha!" on its own plays her laugh sound (`[laugh]` through speak); "Ha, ..." opening
  a sentence is dropped.
- Linda's block (`LINDA`): warm, one to three short sentences, food, saints, Italy; only what is
  surely true, no made-up memories, scam guard ("check with Diane or Steve"), friendly on politics
  without passing on claims.
- Local model: Qwen2.5-14B-Instruct Q4_K_M on JEDIPC (llama-server `-np 2 -c 8192 -fa on -ub 128`,
  `cache_prompt`), `max_tokens` 160, temperature 0.7, `memory_turns` 6, `memory_timeout_s` 600.

### Voice (`speak.py`)
- Kokoro on H2-Host (`tts_url` port 8092, `tts_voice` nicole, speed 1.0): 0.26 s for a short line,
  about 0.21 s per second of speech. Piper (vits-piper-en_US-hfc_female-medium) on Rosie when it is off.
- **"H2 is off" mode**: the server is checked in the background every 5 s (`SERVER_CHECK_S`), never on
  the way to speaking; it switches both ways by itself and logs each switch once.
- `lead_s` 0.08, `pause_s` 0.2, every short line cached in `~/voice/tts_cache`.

## Known limits
- Steve Memory's background analysis shares the JEDIPC model; when it is busy a reply can take
  2-4 s to start (seen once, switching from Linda to Diane).
- Switching between people (Linda <-> Diane) re-reads the conversation once (~700 tokens, ~1 s).

Commits (jetnano_robot): b0ab7b1, 4904cbb, 8880fbb, 3a796ce, 0975e6c, d81e12a.
