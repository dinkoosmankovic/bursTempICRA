template = '''
environment {{
  robot_filename: "../{world_case}/robot/solid.robot"
  environment_filename: "../{world_case}/environment/obstacles_{difficulty}.obj"
  max_underestimate: 5.0
}}
generator {{
  seed: {seed}
}}
index {{
  type: {index_type}
  index_params {{
    trees: 8
  }}
  search_params {{
    checks: 128
    use_heap: false
  }}
}}
tree {{
  type: {tree_type}
  use_extended_bubbles: {use_extended_bubbles}
}}
source {{
{source_q_string}
}}
destination {{
{destination_q_string}
}}
'''

robot_type_string = {
    'BUBBLE-true': 'setup.robot',
    'BUBBLE-false': 'solid.robot',
    'BERTRAM_BUBBLE-true': 'setup.robot',
    'BERTRAM_BUBBLE-false': 'solid.robot',
    'CLASSIC-true': 'solid.robot',
    'CLASSIC-false': 'solid.robot',
    'RBT-true': 'solid.robot',
    'RBT-false': 'solid.robot',
    'GRBT-true': 'solid.robot',
    'GRBT-false': 'solid.robot',
}
def parameter_provider():
    for difficulty in ['trivial', 'easy', 'medium', 'hard', 'hardsingle', 'easy2x', 'easy4x', 'easy8x', 'easy16x', 'easy32x', 'easy64x']:
        for index_type in ['KD_TREE']:
            for tree_type in ['CLASSIC', 'RBT', 'GRBT']:
                for use_extended_bubbles in ['true', 'false']:
                    if (use_extended_bubbles is 'true'
                            and tree_type is 'CLASSIC'):
                        continue
                    for seed in range(0, 100):
                        yield {
                            '1_difficulty': difficulty,
                            '6_seed': str(400 * seed),
                            '3_index_type': index_type,
                            '4_tree_type': tree_type,
                            '5_use_extended_bubbles': use_extended_bubbles,
                        }

def generate_files(q_src, q_dst, world_case):
    counter = 0
    for mapping in parameter_provider():
	#print(mapping)
        counter += 1
        print 'At: {}'.format(counter)
        with open('generated_solid___' + world_case + "___" +
                ('___'.join(zip(*sorted(mapping.items()))[1])) +
                '.task', 'w') as f:
            casemap = {
                'world_case': world_case,
                'difficulty': mapping['1_difficulty'],
                'seed': mapping['6_seed'],
                'index_type': mapping['3_index_type'],
                'tree_type': mapping['4_tree_type'],
                'use_extended_bubbles': mapping['5_use_extended_bubbles'],
            }
            filemap = {
                'world_case': world_case,
                'difficulty': mapping['1_difficulty'],
                'seed': mapping['6_seed'],
                'index_type': mapping['3_index_type'],
                'tree_type': mapping['4_tree_type'],
                'use_extended_bubbles': mapping['5_use_extended_bubbles'],
                'source_q_string': q_src[mapping['1_difficulty']],
                'destination_q_string': q_dst[mapping['1_difficulty']],
            }
            f.write(template.format(**filemap))
