echo "STARTING RBT TESTS"
BUR_SIZE=$1
echo "BUR SIZE=$BUR_SIZE"
echo "3 LINK"
mkdir -p ../tests/3_link/
echo "GETTING TIMES, NODES AND C/D FOR 3 LINK EASY "
./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-3-link___easy___KD_TREE___RBT___false___*.task >> ../tests/3_link/rbt_3_link_easy_$BUR_SIZE 
#echo "GETTING PATH"
#./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-3-link___easy___KD_TREE___RBT___false___0.task > ../tests/3_link/rbt_3_link_easy_path_$BUR_SIZE


#echo "GETTING TIMES, NODES AND C/D FOR 3 LINK HARD"
#./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-3-link___hard___KD_TREE___RBT___false___*.task >> ../tests/3_link/rbt_3_link_hard_$BUR_SIZE
#echo "GETTING PATH"
#./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-3-link___hard___KD_TREE___RBT___false___0.task > ../tests/3_link/rbt_3_link_hard_path_$BUR_SIZE

#echo "8 LINK"
# # GETTING TIMES, NODES AND C/D FOR 8 LINK EASY
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-8-link___easy___KD_TREE___RBT___false___*.task >> ../tests/8_link/rbt_8_link_easy_$BUR_SIZE
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-8-link___easy___KD_TREE___RBT___false___0.task > ../tests/8_link/rbt_8_link_easy_path_$BUR_SIZE

# # GETTING TIMES, NODES AND C/D FOR 8 LINK HARD
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-8-link___hard___KD_TREE___RBT___false___*.task >> ../tests/8_link/rbt_8_link_hard_$BUR_SIZE
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-8-link___hard___KD_TREE___RBT___false___0.task > ../tests/8_link/rbt_8_link_hard_path_$BUR_SIZE

 echo "ABB"
# echo "GETTING TIMES, NODES AND C/D FOR ABB EASY"
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___abb-irb-120___easy___KD_TREE___RBT___false___*.task >> ../tests/abb/rbt_abb_easy_$BUR_SIZE
# GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___abb-irb-120___easy___KD_TREE___RBT___false___0.task > ../tests/abb/rbt_abb_easy_path_$BUR_SIZE

 echo "GETTING TIMES, NODES AND C/D FOR ABB HARD"
 ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___abb-irb-120___hard___KD_TREE___RBT___false___0.task >> ../tests/abb/rbt_abb_hard_$BUR_SIZE
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___abb-irb-120___hard___KD_TREE___RBT___false___0.task > ../tests/abb/rbt_abb_hard_path_$BUR_SIZE

echo "FINISHED RBT TESTS"
