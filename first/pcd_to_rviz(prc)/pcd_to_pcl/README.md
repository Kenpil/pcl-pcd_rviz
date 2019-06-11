クラスタリングができているらしい
なぜかコンパイル時に
/usr/include/pcl-1.8/pcl/sample_consensus/model_types.h:99:3: warning: ‘pcl::SAC_SAMPLE_SIZE’ is deprecated: This map is deprecated and is kept only to prevent breaking existing user code. Starting from PCL 1.8.0 model sample size is a protected member of the SampleConsensusModel class [-Wdeprecated-declarations]
   SAC_SAMPLE_SIZE (sample_size_pairs, sample_size_pairs + sizeof (sample_size_pairs) / sizeof (SampleSizeModel));
と言われるが、今の所問題なし
