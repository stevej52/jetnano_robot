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

"""The word rules behind turn-taking and answering (2026-09-28): is he done, does the
question go straight to Claude, and where her voice may start."""

from jetnano_bringup.brain import fact_question, sentences
from jetnano_bringup.turn import unfinished
import pytest


@pytest.mark.parametrize('text, why', [
    ('I went to the store.', None),
    ('I went to the', 'word'),
    ('can you tell me about my', 'word'),
    ('so I was thinking and', 'word'),
    ('so, um', 'filler'),
    ('Well', 'filler'),
    ('I was thinking,', 'mark'),
    ('and then -', 'mark'),
    ('let me see...', 'mark'),
    ("what's the weather like?", 'word'),       # a 'word' a question mark or his voice overrules
    ('Rosie, how are you', None),
    ('', None),
    ('   ', None),
])
def test_unfinished(text, why):
    assert unfinished(text) == why


@pytest.mark.parametrize('text', [
    'Who won the World Series in 1988?',
    'Rosie, who won the world series?',
    "What's the capital of France?",
    'How many legs does a spider have?',
    'Tell me about the Roman empire',
    'When did the Titanic sink?',
    'um, why is the sky blue?',
])
def test_fact_question_goes_to_claude(text):
    assert fact_question(text)


@pytest.mark.parametrize('text', [
    'How are you?',
    "What's the weather?",
    'Tell me about Sadie',
    'How big is my house?',
    'What time is it?',
    'Turn left',
    'Who is Queenie?',
    'How old are you, Rosie?',
])
def test_not_a_fact_question(text):
    assert not fact_question(text)


def test_sentences_whole():
    out, rest = sentences('The Dodgers won last night, and then it rained. More')
    assert out == ['The Dodgers won last night, and then it rained.']
    assert rest == 'More'


def test_sentences_first_clause_starts_her_voice():
    out, rest = sentences('The Dodgers won last night, and then it rained. More', clause=True)
    assert out == ['The Dodgers won last night,', 'and then it rained.']
    assert rest == 'More'


def test_sentences_short_clause_waits():
    # under CLAUSE_WORDS words the comma is not enough: wait for the full stop
    out, rest = sentences('Yes, it did. And', clause=True)
    assert out == ['Yes, it did.']
    assert rest == 'And'


def test_sentences_only_the_first_clause_is_early():
    out, rest = sentences('Well I think so, maybe. Then, after that, it', clause=True)
    assert out == ['Well I think so,', 'maybe.']
    assert rest == 'Then, after that, it'


def test_sentences_needs_the_space_after():
    assert sentences('Hello there') == ([], 'Hello there')
    assert sentences('Hi.') == ([], 'Hi.')
    assert sentences('He said "go." Then') == (['He said "go."'], 'Then')
