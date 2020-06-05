#!/usr/bin/python

from generator_base import generate_files

src_q_string_2 = {
    'trivial': '''
  q:  45
  q:  45
 ''',
    'easy': '''
  q: 45
  q:  45
 ''',
    'medium': '''
  q:  90
  q:   0
  ''',
    'hard': '''
  q: 115
  q: 57
  ''',
   'hardsingle': '''
  q: 115
  q:  57
  ''',
}

dst_q_string_2 = {
    'trivial': '''
  q: -45
  q: -45
 ''',
    'easy': '''
  q: 135
  q: -45
  ''',
    'medium': '''
  q:   0
  q:   0
  ''',
    'hard': '''
  q: 65
  q: -57
  ''',
  'hardsingle': '''
  q: 65
  q: -57
  ''',
}

src_q_string_3 = {
    'trivial': '''
  q:  45
  q:  45
  q:  45
''',
    'easy': '''
  q:  45
  q:  45
  q:  45
''',
    'medium': '''
  q:  90
  q:   0
  q:  90
''',
    'hard': '''
  q:  90
  q:   0
  q:  90
''',
}

dst_q_string_3 = {
    'trivial': '''
  q: -45
  q: -45
  q: -45
''',
    'easy': '''
  q: -45
  q: -45
  q: -45
''',
    'medium': '''
  q:   0
  q:   0
  q: -90
''',
    'hard': '''
  q:   0
  q:   0
  q: -90
''',
}

src_q_string_6 = {
    'trivial': '''
  q: -45
  q:  39
  q:   6
  q:   9
  q:   9
  q:   9
''',
    'easy': '''
  q: -60
  q:  39
  q:   6
  q:   9
  q:   9
  q:   9
''',
    'easy2x': '''
  q: -60
  q:  39
  q:   6
  q:   9
  q:   9
  q:   9
''',
    'easy4x': '''
  q: -60
  q:  39
  q:   6
  q:   9
  q:   9
  q:   9
''',
    'easy8x': '''
  q: -60
  q:  39
  q:   6
  q:   9
  q:   9
  q:   9
''',
    'easy16x': '''
  q: -60
  q:  39
  q:   6
  q:   9
  q:   9
  q:   9
''',
    'easy32x': '''
  q: -60
  q:  39
  q:   6
  q:   9
  q:   9
  q:   9
''',
    'easy64x': '''
  q: -60
  q:  39
  q:   6
  q:   9
  q:   9
  q:   9
''',
    'medium': '''
  q: -90
  q:  50
  q: -50
  q:   0
  q:   0
  q:   0
''',
    'hard': '''
  q: 30
  q:  90
  q: -70
  q:   0
  q:   0
  q:   0
''',
    'hardsingle': '''
  q: 30
  q:  90
  q: -70
  q:   0
  q:   0
  q:   0
''',
}

dst_q_string_6 = {
    'trivial': '''
  q:  45
  q:  21
  q:  21
  q:  -9
  q:  -9
  q:  -9
''',
    'easy': '''
  q:  60
  q:  21
  q:  21
  q:  -9
  q:  -9
  q:  -9
''',
    'easy2x': '''
  q:  60
  q:  21
  q:  21
  q:  -9
  q:  -9
  q:  -9
''',
    'easy4x': '''
  q:  60
  q:  21
  q:  21
  q:  -9
  q:  -9
  q:  -9
''',
    'medium': '''
  q:   0
  q:  90
  q: -90
  q:  -9
  q:  -9
  q:  -9
''',
    'easy8x': '''
  q:  60
  q:  21
  q:  21
  q:  -9
  q:  -9
  q:  -9
''',
    'easy16x': '''
  q:  60
  q:  21
  q:  21
  q:  -9
  q:  -9
  q:  -9
''',
    'easy32x': '''
  q:  60
  q:  21
  q:  21
  q:  -9
  q:  -9
  q:  -9
''',
    'easy64x': '''
  q:  60
  q:  21
  q:  21
  q:  -9
  q:  -9
  q:  -9
''',
    'hard': '''
  q: -30
  q:  90
  q: -70
  q:   0
  q:   0
  q:   0
''',
    'hardsingle': '''
  q: -30
  q:  90
  q: -70
  q:   0
  q:   0
  q:   0
''',
}

src_q_string_8 = {
    'trivial': '''
  q:  10
  q:  10
  q:  10
  q:  10
  q:  10
  q:  10
  q:  10
  q:  10
''',
    'easy': '''
  q:  95.729578
  q:  14.323945
  q:  14.323945
  q:  14.323945
  q:  14.323945
  q:  -14.323945
  q:  -14.323945
  q:  -14.323945
''',
    'medium': '''
  q:   0
  q:   0
  q:   0
  q:  90
  q:   0
  q:   0
  q:  90
  q:   0
''',
    'hard': '''
  q:   95.729578
  q:   14.323945
  q:   14.323945
  q:  14.323945
  q:  14.323945
  q:  14.323945
  q:  14.323945
  q:  14.323945
''',
    'hardsingle': '''
  q:   95.729578
  q:   14.323945
  q:   14.323945
  q:  14.323945
  q:  14.323945
  q:  14.323945
  q:  14.323945
  q:  14.323945
''',
}

dst_q_string_8 = {
    'trivial': '''
  q: -10
  q: -10
  q: -10
  q: -10
  q: -10
  q: -10
  q: -10
  q: -10
''',
    'easy': '''
  q: 84.270422
  q: -14.323945
  q: -14.323945
  q: -14.323945
  q: -14.323945
  q: 14.323945
  q: 14.323945
  q: 14.323945
''',
    'medium': '''
  q:   0
  q:   0
  q:   0
  q: -90
  q:   0
  q:   0
  q: -90
  q:   0
''',
    'hard': '''
  q:   84.270422
  q:   -14.323945
  q:   -14.323945
  q:   -14.323945
  q:   -14.323945
  q:   -14.323945
  q:   -14.323945
  q:   -14.323945
''',
   'hardsingle': '''
  q:   84.270422
  q:   -14.323945
  q:   -14.323945
  q:   -14.323945
  q:   -14.323945
  q:   -14.323945
  q:   -14.323945
  q:   -14.323945
''',
}

if __name__ == '__main__':
    generate_files(src_q_string_6, dst_q_string_6, 'abb-irb-120')
    #generate_files(src_q_string_3, dst_q_string_3, 'generic-3-link')
    #generate_files(src_q_string_2, dst_q_string_2, 'generic-2-link')
    #generate_files(src_q_string_8, dst_q_string_8, 'generic-8-link')
