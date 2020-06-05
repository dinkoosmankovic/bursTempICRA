#!/bin/bash

#rm ../tests/3_link/*
#rm ../tests/8_link/*
#rm ../tests/abb/*

# BUBBLE
echo "STARTING BUBBLE TESTS"
echo "3 LINK"
# # GETTING TIMES, NODES AND C/D FOR 3 LINK EASY 
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-3-link___easy___KD_TREE___BUBBLE___false___*.task >> ../tests/3_link/bubble_3_link_easy 
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-3-link___easy___KD_TREE___BUBBLE___false___0.task > ../tests/3_link/bubble_3_link_easy_path

# # GETTING TIMES, NODES AND C/D FOR 3 LINK HARD
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-3-link___hard___KD_TREE___BUBBLE___false___*.task >> ../tests/3_link/bubble_3_link_hard 
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-3-link___hard___KD_TREE___BUBBLE___false___0.task > ../tests/3_link/bubble_3_link_hard_path

# echo "8 LINK"
# # GETTING TIMES, NODES AND C/D FOR 8 LINK EASY
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-8-link___easy___KD_TREE___BUBBLE___false___*.task >> ../tests/8_link/bubble_8_link_easy
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-8-link___easy___KD_TREE___BUBBLE___false___0.task > ../tests/8_link/bubble_8_link_easy_path

# # GETTING TIMES, NODES AND C/D FOR 8 LINK HARD
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-8-link___hard___KD_TREE___BUBBLE___false___*.task >> ../tests/8_link/bubble_8_link_hard
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-8-link___hard___KD_TREE___BUBBLE___false___0.task > ../tests/8_link/bubble_8_link_hard_path

# echo "ABB"
# # GETTING TIMES, NODES AND C/D FOR ABB EASY
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___abb-irb-120___easy___KD_TREE___BUBBLE___false___*.task >> ../tests/abb/bubble_abb_easy
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___abb-irb-120___easy___KD_TREE___BUBBLE___false___0.task > ../tests/abb/bubble_abb_easy_path

# # GETTING TIMES, NODES AND C/D FOR ABB HARD
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___abb-irb-120___hard___KD_TREE___BUBBLE___false___*.task >> ../tests/abb/bubble_abb_hard
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___abb-irb-120___hard___KD_TREE___BUBBLE___false___0.task > ../tests/abb/bubble_abb_hard_path

# echo "FINISHED BUBBLE TESTS"


# CLASSIC
echo "STARTING CLASSIC TESTS"
# echo "3 LINK"
# # GETTING TIMES, NODES AND C/D FOR 3 LINK EASY 
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-3-link___easy___KD_TREE___CLASSIC___false___*.task >> ../tests/3_link/classic_3_link_easy 
# # GETTING PATH
 #./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-3-link___easy___KD_TREE___CLASSIC___false___0.task > ../tests/3_link/classic_3_link_easy_path

# # GETTING TIMES, NODES AND C/D FOR 3 LINK HARD
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-3-link___hard___KD_TREE___CLASSIC___false___*.task >> ../tests/3_link/classic_3_link_hard 
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-3-link___hard___KD_TREE___CLASSIC___false___0.task > ../tests/3_link/classic_3_link_hard_path

# echo "8 LINK"
# # GETTING TIMES, NODES AND C/D FOR 8 LINK EASY
#./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-8-link___easy___KD_TREE___CLASSIC___false___*.task >> ../tests/8_link/classic_8_link_easy
# # GETTING PATH
#./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-8-link___easy___KD_TREE___CLASSIC___false___0.task > ../tests/8_link/classic_8_link_easy_path

# # GETTING TIMES, NODES AND C/D FOR 8 LINK HARD
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-8-link___hard___KD_TREE___CLASSIC___false___*.task >> ../tests/8_link/classic_8_link_hard
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-8-link___hard___KD_TREE___CLASSIC___false___0.task > ../tests/8_link/classic_8_link_hard_path

# echo "ABB"
# # GETTING TIMES, NODES AND C/D FOR ABB EASY
#./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___abb-irb-120___easy___KD_TREE___CLASSIC___false___1*00.task >> ../tests/abb/classic_abb_easy
# # GETTING PATH
#./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___abb-irb-120___easy___KD_TREE___CLASSIC___false___0.task > ../tests/abb/classic_abb_easy_path

# # GETTING TIMES, NODES AND C/D FOR ABB HARD
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___abb-irb-120___hard___KD_TREE___CLASSIC___false___*.task >> ../tests/abb/classic_abb_hard
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___abb-irb-120___hard___KD_TREE___CLASSIC___false___0.task > ../tests/abb/classic_abb_hard_path

# echo "FINISHED CLASSIC TESTS"

# # BERTRAM_BUBBLE
echo "STARTING BERTRAM_BUBBLE TESTS"
echo "3 LINK"
# # GETTING TIMES, NODES AND C/D FOR 3 LINK EASY 
 ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-3-link___easy___KD_TREE___BERTRAM_BUBBLE___false___1*00.task >> ../tests/3_link/bertram_3_link_easy 
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-3-link___easy___KD_TREE___BERTRAM_BUBBLE___false___0.task > ../tests/3_link/bertram_3_link_easy_path

# # GETTING TIMES, NODES AND C/D FOR 3 LINK HARD
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-3-link___hard___KD_TREE___BERTRAM_BUBBLE___false___*.task >> ../tests/3_link/bertram_3_link_hard 
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-3-link___hard___KD_TREE___BERTRAM_BUBBLE___false___0.task > ../tests/3_link/bertram_3_link_hard_path

# echo "8 LINK"
# # GETTING TIMES, NODES AND C/D FOR 8 LINK EASY
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-8-link___easy___KD_TREE___BERTRAM_BUBBLE___false___*.task >> ../tests/8_link/bertram_8_link_easy
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-8-link___easy___KD_TREE___BERTRAM_BUBBLE___false___0.task > ../tests/8_link/bertram_8_link_easy_path

# # GETTING TIMES, NODES AND C/D FOR 8 LINK HARD
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___generic-8-link___hard___KD_TREE___BERTRAM_BUBBLE___false___*.task >> ../tests/8_link/bertram_8_link_hard
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___generic-8-link___hard___KD_TREE___BERTRAM_BUBBLE___false___0.task > ../tests/8_link/bertram_8_link_hard_path

# echo "ABB"
# # GETTING TIMES, NODES AND C/D FOR ABB EASY
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___abb-irb-120___easy___KD_TREE___BERTRAM_BUBBLE___false___1*200.task >> ../tests/abb/bertram_abb_easy
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___abb-irb-120___easy___KD_TREE___BERTRAM_BUBBLE___false___0.task > ../tests/abb/bertram_abb_easy_path

# # GETTING TIMES, NODES AND C/D FOR ABB HARD
# ./bubblesrrt -logtostderr=false --output_type=times ../motion-planning-data/generated/generated_solid___abb-irb-120___hard___KD_TREE___BERTRAM_BUBBLE___false___1*200.task >> ../tests/abb/bertram_abb_hard
# # GETTING PATH
# ./bubblesrrt -logtostderr=false --output_type=path ../motion-planning-data/generated/generated_solid___abb-irb-120___hard___KD_TREE___BERTRAM_BUBBLE___false___0.task > ../tests/abb/bertram_abb_hard_path

# echo "FINISHED BERTRAM_BUBBLE TESTS"