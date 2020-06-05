echo "STARTING RBT/GRBT TESTS"
echo $1 LINK

#mkdir -p ../tests/$1_link/
#echo "GETTING TIMES, NODES AND C/D FOR $1 LINK $2 RBT"
#rm ../tests/$1_link/rbt_$1_link_$2
#rm ../tests/$1_link/grbt_$1_link_$2
#./bubblesrrt -logtostderr=true --output_type=times ../motion-planning-data/generated/generated_solid___generic-$1-link___$2___KD_TREE___RBT___false___1*.task >> ../tests/$1_link/rbt_$1_link_$2
#echo "GETTING TIMES, NODES AND C/D FOR $1 LINK $2 GRBT"
#./bubblesrrt -logtostderr=true --output_type=times ../motion-planning-data/generated/generated_solid___generic-$1-link___$2___KD_TREE___GRBT___false___1*.task >> ../tests/$1_link/grbt_$1_link_$2

#./bubblesrrt -logtostderr=true --output_type=path ../motion-planning-data/generated/generated_solid___abb-irb-120___easy___KD_TREE___GRBT___false___0.task > ../tests/abb/grbt_abb_easy_path
#./bubblesrrt -logtostderr=true --output_type=times ../motion-planning-data/generated/generated_solid___abb-irb-120___easy___KD_TREE___GRBT___false___0.task >> ../tests/abb/abb_grbt_easy
#rm ../tests/$1/rbt_$1_link_$2

for f in ../motion-planning-data/generated/generated_solid___$1___$2___KD_TREE___RBT___false___*.task; do
    echo "Working with ${f}"
    timeout 15s ./bubblesrrt -logtostderr=true --output_type=times ${f} >> ../tests/$1/rbt_$1_link_$2
done;

# echo "FINISHED RBT/GRBT TESTS"


#./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___abb-irb-120___hard___KD_TREE___GRBT___false___0.task >> ../tests/abb-irb-120/abb_grbt_hard_path