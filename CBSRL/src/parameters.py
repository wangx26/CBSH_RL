LR                  = 1e-4
gamma               = 0.9
tau                 = 1.0
beta                = 0.01
num_local_steps     = 500
num_global_steps    = 2e4 #5e5
num_processes       = 2
save_interval       = 200 #500
max_actions         = 50

log_path            = "tensorboard/CBSRL"
save_path           = "trained_models"

use_gpu             = True
load_from_pre       = False