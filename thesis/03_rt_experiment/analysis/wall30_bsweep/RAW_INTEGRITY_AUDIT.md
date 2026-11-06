# wall30 B-sweep READ-ONLY raw integrity audit

Overall: **PASS** (24/24 runs). No raw files were modified, regenerated, overwritten, cleaned, or renamed.

| config | run | tag | files | Sonar rows | Wheel rows | raw TIM2 | mono S/W | dups S/W | ovf S/W | sens_err | freq vs n | result |
|--------|-----|-----|-------|------------|------------|----------|----------|----------|---------|----------|-----------|--------|
| noattest | 1 | `phase2r_noattest_r1` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| noattest | 2 | `phase2r_noattest_r2` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| noattest | 3 | `phase2r_noattest_r3` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B64 | 1 | `phase3r_B64_r1` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B64 | 2 | `phase3r_B64_r2` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B64 | 3 | `phase3r_B64_r3` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B128 | 1 | `phase3r_B128_r1` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B128 | 2 | `phase3r_B128_r2` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B128 | 3 | `phase3r_B128_r3` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B256 | 1 | `phase3r_B256_r1` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B256 | 2 | `phase3r_B256_r2` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B256 | 3 | `phase3r_B256_r3` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B512 | 1 | `phase3r_B512_r1` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B512 | 2 | `phase3r_B512_r2` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B512 | 3 | `phase3r_B512_r3` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B1024 | 1 | `phase3r_B1024_r1` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B1024 | 2 | `phase3r_B1024_r2` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B1024 | 3 | `phase3r_B1024_r3` | OK | 300 | 300 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B2048 | 1 | `phase3r_B2048_r1` | OK | 164 | 164 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B2048 | 2 | `phase3r_B2048_r2` | OK | 164 | 164 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B2048 | 3 | `phase3r_B2048_r3` | OK | 164 | 164 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B4096 | 1 | `phase3r_B4096_r1` | OK | 84 | 84 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B4096 | 2 | `phase3r_B4096_r2` | OK | 84 | 84 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |
| B4096 | 3 | `phase3r_B4096_r3` | OK | 84 | 84 | OK | True/True | 0/0 | 0/0 | 0 | OK | **PASS** |

## Issues detail
- none

## Notes

- Sonar TIM2 columns checked: `actual_start_us`, `finish_us`.
- Wheel TIM2 columns checked: `window_open_us`, `s_exec_us`/`window_end_us`, `finish_us`.
- Frequency check: CSV row count vs UART `sonar_starts`/`wheel_starts`/`freq_mHz` and TIM2 window.
- SHA256 below covers every existing raw companion file for the 24 tags.

## SHA256 manifest (raw files)

| file | bytes | sha256 |
|------|-------|--------|
| `phase2r_noattest_r1_uart_capture.txt` | 80313 | `7c8b03c8c874f9f057d2330abed2de5b8ba2576483a8733c5fecc07565820727` |
| `phase2r_noattest_r1_sonar_timing.csv` | 35791 | `1b26acbf8cc63cb2c6ff8e912ea47727ad80fda59d60a6a18527e8bb318cc41b` |
| `phase2r_noattest_r1_wheel_timing.csv` | 40868 | `9fe7cc864fd518f8fa6fab9b1e423be9cdf4393fc60cfe2cd69670db3d5d1fc6` |
| `phase2r_noattest_r1_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase2r_noattest_r1_run_meta.txt` | 260 | `a77b9c98fad4e6efcc5856ab41064c3a15c4fc547cf60776faa92f6f3a121274` |
| `phase2r_noattest_r2_uart_capture.txt` | 80333 | `6669e23b8476a1e5b4d886d3d65ba5c98e6ac65b5841d0e9db0fa241bba29ed4` |
| `phase2r_noattest_r2_sonar_timing.csv` | 35811 | `944182c434f2d37f15b9471c3cccb34f018154a384c623e2da948167e29b564c` |
| `phase2r_noattest_r2_wheel_timing.csv` | 40868 | `9fe7cc864fd518f8fa6fab9b1e423be9cdf4393fc60cfe2cd69670db3d5d1fc6` |
| `phase2r_noattest_r2_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase2r_noattest_r2_run_meta.txt` | 260 | `37c85d797500281376041a09742429147ba6cfbb3a1093cc0bac2d342b843964` |
| `phase2r_noattest_r3_uart_capture.txt` | 80332 | `0a6c2518b37e68463aa4e6cfa8f50b6b7b3fe8dd1c56e2754aca9ae0c51e00ed` |
| `phase2r_noattest_r3_sonar_timing.csv` | 35810 | `790fa87d8e010f771b4833ebe057257058d66928bf5ced00be7d79475a1896ac` |
| `phase2r_noattest_r3_wheel_timing.csv` | 40868 | `9fe7cc864fd518f8fa6fab9b1e423be9cdf4393fc60cfe2cd69670db3d5d1fc6` |
| `phase2r_noattest_r3_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase2r_noattest_r3_run_meta.txt` | 260 | `37c85d797500281376041a09742429147ba6cfbb3a1093cc0bac2d342b843964` |
| `phase3r_B64_r1_uart_capture.txt` | 81138 | `23f131b195223b30987ff0f74097f6c9776d61f25b48545c8ba07b0c1d06981a` |
| `phase3r_B64_r1_sonar_timing.csv` | 35951 | `8e18849cc6b90c0beff181bf3abe44e23a6b5af8988ddd066ba6f2364671e976` |
| `phase3r_B64_r1_wheel_timing.csv` | 41208 | `6430e64d6244310ecb35a0d104932c8ad8007ae08573e06f8083fafa67217594` |
| `phase3r_B64_r1_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B64_r1_run_meta.txt` | 258 | `88510e65fefea8311aaf33acdf1cd1e8ff61b53022e6f02d8a1bd16ec2fcfd7f` |
| `phase3r_B64_r2_uart_capture.txt` | 81138 | `022559b8b5f864e67243b9f8fd59c0d7018f896ba78f61735567fa1dbd16f6bf` |
| `phase3r_B64_r2_sonar_timing.csv` | 35947 | `927394ca3c3edd54c1a2e3313d59d008469a8c3d7768128b02876c7f20f75eda` |
| `phase3r_B64_r2_wheel_timing.csv` | 41212 | `4e29ad90b21d413dd164486c98f8aa24cd880c4255be6e14da49e9c7fae3bf2b` |
| `phase3r_B64_r2_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B64_r2_run_meta.txt` | 258 | `88510e65fefea8311aaf33acdf1cd1e8ff61b53022e6f02d8a1bd16ec2fcfd7f` |
| `phase3r_B64_r3_uart_capture.txt` | 81118 | `f33d564b1c3759fd60573a865733e7c138f54ec69fb96909587b2489eb8917dd` |
| `phase3r_B64_r3_sonar_timing.csv` | 35948 | `a8ffae92ebefe6b5154822ae3e62fe07d931ef1f4a923d16990f726ebaa22581` |
| `phase3r_B64_r3_wheel_timing.csv` | 41191 | `3fb5ba51cc648ab93504aff50b0af02857a41c1255eda6d4731db9d3e77ac4d3` |
| `phase3r_B64_r3_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B64_r3_run_meta.txt` | 258 | `88510e65fefea8311aaf33acdf1cd1e8ff61b53022e6f02d8a1bd16ec2fcfd7f` |
| `phase3r_B128_r1_uart_capture.txt` | 81248 | `ca0478918c13d073eeb3180b917e2ff09d86da9bd6d96a415329f936f9bf9cd1` |
| `phase3r_B128_r1_sonar_timing.csv` | 35969 | `1d33d3420755a5ca3591cbf4121a31aa664ff876784385d6050e933a74421136` |
| `phase3r_B128_r1_wheel_timing.csv` | 41297 | `77cde988a18cb1f4aeada04424f77cc0c87ae23fbd559f8cdbe01b158b06f8a9` |
| `phase3r_B128_r1_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B128_r1_run_meta.txt` | 259 | `5563b3b7c022a61797416c788048dd5fb1fc3de50f1d4cb4a11c8e8ef75fac63` |
| `phase3r_B128_r2_uart_capture.txt` | 81238 | `52f07b0b064f80f58ca6ba53c6488ffc54cb72eba43593e3d4b8121070285322` |
| `phase3r_B128_r2_sonar_timing.csv` | 35965 | `f30ab666b970244609794d43cfc09b5f68cdd4ab40d46d85b658281886e599af` |
| `phase3r_B128_r2_wheel_timing.csv` | 41291 | `b5157b021b6d9f1efb787c8cd802b4f22f97c53720c987e489d56528d561d8c0` |
| `phase3r_B128_r2_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B128_r2_run_meta.txt` | 259 | `5563b3b7c022a61797416c788048dd5fb1fc3de50f1d4cb4a11c8e8ef75fac63` |
| `phase3r_B128_r3_uart_capture.txt` | 81248 | `c5b3b48feedec7bc809570ae614723cb6d17deaf440a75affe77419387fbb60c` |
| `phase3r_B128_r3_sonar_timing.csv` | 35966 | `4100ef3c449a3e9c49e44f41d593e919779112c05f7daf255fb46d04cb12c972` |
| `phase3r_B128_r3_wheel_timing.csv` | 41300 | `7df402a725a276d52e5c827766336bc9bccf1832af57d1cbbbc65b06a5099bdb` |
| `phase3r_B128_r3_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B128_r3_run_meta.txt` | 259 | `5563b3b7c022a61797416c788048dd5fb1fc3de50f1d4cb4a11c8e8ef75fac63` |
| `phase3r_B256_r1_uart_capture.txt` | 81513 | `ed13509259c9b8011c15c8dc74d1179d9480b358920b04cc01fa8901c6438ee4` |
| `phase3r_B256_r1_sonar_timing.csv` | 36069 | `352819370dee492c4a35a4fb8a8bc3cde723f595cb3af69ee8d484e21bb6e897` |
| `phase3r_B256_r1_wheel_timing.csv` | 41462 | `70e8700cda6f7d9a4e6430f5abce8abed751f65635a3856020fcf1df558c2d2e` |
| `phase3r_B256_r1_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B256_r1_run_meta.txt` | 259 | `b969473bc708c39687eebaa8afae171dc8d1712ee37a6f2c953ffde31b6b3568` |
| `phase3r_B256_r2_uart_capture.txt` | 81514 | `8ac73707882077d7c5bb051a5ac3ef1b355a7bc2b88203068c2d20297293f276` |
| `phase3r_B256_r2_sonar_timing.csv` | 36070 | `aa69347b2f38feb506cf390cd0a2b6e3ae9d0bdabc859e04a4c3ec1814abde3d` |
| `phase3r_B256_r2_wheel_timing.csv` | 41462 | `14f2a46ae3adb7e261c232e2880e76f3aa545cf73627a4f8fe16da002ba637f2` |
| `phase3r_B256_r2_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B256_r2_run_meta.txt` | 259 | `b969473bc708c39687eebaa8afae171dc8d1712ee37a6f2c953ffde31b6b3568` |
| `phase3r_B256_r3_uart_capture.txt` | 81532 | `4068ab93380d689db6c7c1d0d09ed001c4ee28bc1b565ed95b951cb1c59c1a19` |
| `phase3r_B256_r3_sonar_timing.csv` | 36075 | `6004646403e17258bfe49f63c37c94accfafca85be9d4b97eb12e5b2ab41d16a` |
| `phase3r_B256_r3_wheel_timing.csv` | 41475 | `ad88c84f2b129c86ccc51bd529d6d49ea5b2a1e7448859b3fa79cb0e47e17e3f` |
| `phase3r_B256_r3_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B256_r3_run_meta.txt` | 259 | `b969473bc708c39687eebaa8afae171dc8d1712ee37a6f2c953ffde31b6b3568` |
| `phase3r_B512_r1_uart_capture.txt` | 82149 | `5a25d1332321d9552874d856f9888f75334b2a39ad998d4e0940e5d05e36793d` |
| `phase3r_B512_r1_sonar_timing.csv` | 36343 | `c8faf9b39b82a57b915e92e778453ffc5b70175a9fb6753afae73c9fdc95a6ad` |
| `phase3r_B512_r1_wheel_timing.csv` | 41824 | `1e4f61e4bc6001d32f61cff17c830cf21a45c359a827c72d35cc22100739a388` |
| `phase3r_B512_r1_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B512_r1_run_meta.txt` | 259 | `84dd151f063ef3aa69a6203f832cba46a4a4699469b98388eea1eb923cd4d4d4` |
| `phase3r_B512_r2_uart_capture.txt` | 82150 | `d8632c8595f22f9b76d97243225f4bb3b726dd0a27f88ced9d761399db5a2510` |
| `phase3r_B512_r2_sonar_timing.csv` | 36343 | `6432f0049fef47db42a03a931638792001e26c159c90f67aa12c79e9b8e28845` |
| `phase3r_B512_r2_wheel_timing.csv` | 41825 | `32e566118ee54aa1f5820ca26b7005bb333d60ef5f634dfe30e182c8ba9b605d` |
| `phase3r_B512_r2_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B512_r2_run_meta.txt` | 259 | `84dd151f063ef3aa69a6203f832cba46a4a4699469b98388eea1eb923cd4d4d4` |
| `phase3r_B512_r3_uart_capture.txt` | 82158 | `85e20babec30bd0605902dc784f936cd215be6da2d3e719ace85964c389d1108` |
| `phase3r_B512_r3_sonar_timing.csv` | 36343 | `75b72a318e8d4e2fa11cd639ff46fcf103a8e84ca047c584606df135275d5c0a` |
| `phase3r_B512_r3_wheel_timing.csv` | 41833 | `13ee06cd39d48ddc3985b9baa65e792ed255df0b0405684c73f29fc6707ce8b1` |
| `phase3r_B512_r3_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B512_r3_run_meta.txt` | 259 | `84dd151f063ef3aa69a6203f832cba46a4a4699469b98388eea1eb923cd4d4d4` |
| `phase3r_B1024_r1_uart_capture.txt` | 82108 | `ed6eecd50473469892b034d9a4ee93e893cd257b50613f9c1d9ab618d24a518b` |
| `phase3r_B1024_r1_sonar_timing.csv` | 36327 | `14417e4c6b16b583c1cfdc0162fe26839fbdb72fb7a60232da5bf233c3871ba2` |
| `phase3r_B1024_r1_wheel_timing.csv` | 41795 | `d0e587ea045cadc0396b057d6b35590ad8dbce74e9b7ee635dd360efb95816a1` |
| `phase3r_B1024_r1_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B1024_r1_run_meta.txt` | 260 | `b8eac0679e9be2cb059d20ac7f92b2627f92ca8940af44d91782182bd2b3c18c` |
| `phase3r_B1024_r2_uart_capture.txt` | 82108 | `f532489075bfb1d181e48fb94422723bba169ca4f8acd89c7e1a8994a6586be4` |
| `phase3r_B1024_r2_sonar_timing.csv` | 36327 | `0b67173213f8f4ff111e1b31c2ad8f445dda773b17dfb97f3d16c60d42a27849` |
| `phase3r_B1024_r2_wheel_timing.csv` | 41795 | `62a2407ddce47d0ff8a4f803f5a6275039db6d6d47214f78b8c81223e921b815` |
| `phase3r_B1024_r2_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B1024_r2_run_meta.txt` | 260 | `b8eac0679e9be2cb059d20ac7f92b2627f92ca8940af44d91782182bd2b3c18c` |
| `phase3r_B1024_r3_uart_capture.txt` | 82109 | `277ec65b50bfc5ecbb65d67f6eb2fb3101e024f0988c409f1c2486054fb13dc0` |
| `phase3r_B1024_r3_sonar_timing.csv` | 36327 | `44e4f07c752675cd041261d2f493e1dd228dcbbe849310d4e6f44b1a69f1869f` |
| `phase3r_B1024_r3_wheel_timing.csv` | 41796 | `ce2e3014b38e39c20085a5bdf9f665fcb90a1a0b00e40acc0282dfc5f6561d17` |
| `phase3r_B1024_r3_obs_per_second.csv` | 525 | `81e0790c88413d03708b8e1de4fbd0182c1ec2216e199ccea9c0ffacf0ecb0cb` |
| `phase3r_B1024_r3_run_meta.txt` | 260 | `b8eac0679e9be2cb059d20ac7f92b2627f92ca8940af44d91782182bd2b3c18c` |
| `phase3r_B2048_r1_uart_capture.txt` | 47703 | `891645ae376506d63400d4418adf41aa8266a9321764b9660b77be676aadd0fb` |
| `phase3r_B2048_r1_sonar_timing.csv` | 19861 | `f834356ea8e5c0235c5afd8e119d66223e4ecdeb88c0b4143f5c802944315f10` |
| `phase3r_B2048_r1_wheel_timing.csv` | 24253 | `691fd7bcc7fad0d74949b0b3ad2817e6d3b31ed3d7076147ab80ea654820d4dd` |
| `phase3r_B2048_r1_obs_per_second.csv` | 405 | `958ae3aedbbf761f733162fcb37813cc79b7417a8b2c7051a21c28a3a7d755db` |
| `phase3r_B2048_r1_run_meta.txt` | 260 | `a88f212050fb8a48ed552a4c69c0203cc794d408ef49a7169fc089c18947fb44` |
| `phase3r_B2048_r2_uart_capture.txt` | 47703 | `f8e6bc8e27bc12774b82b2ad081e92b823ac559fcacbfd99ed8f48c5f3c000b1` |
| `phase3r_B2048_r2_sonar_timing.csv` | 19861 | `7f49e959142da4fefaf2f9734ec94bb5c3ce4b04a1528be85c68c5594c4b4e21` |
| `phase3r_B2048_r2_wheel_timing.csv` | 24253 | `4521a2e8e1a90a5fa0a19c0490dec194950b5e5b467c0061449325f5875cd67d` |
| `phase3r_B2048_r2_obs_per_second.csv` | 405 | `7ec2902b9c4713794817496b564d7d397edf3563f11a24d8b64fd99c2255343b` |
| `phase3r_B2048_r2_run_meta.txt` | 260 | `da27bd53671e75579ed72770cf8b4071f6324112083b418ac8c8ff6a903aaefd` |
| `phase3r_B2048_r3_uart_capture.txt` | 47703 | `5c9bcbbf394f0146ecb86007caa7ac6541d0bee94fefddb83b5824052dd54f24` |
| `phase3r_B2048_r3_sonar_timing.csv` | 19861 | `95ed11c98f3041e455ca17ccfb8a9212e15c81056258098b8308d1755cc45aa1` |
| `phase3r_B2048_r3_wheel_timing.csv` | 24253 | `5a509b1b1b3f0ef275df92e56658b68305347e1cd106926f96c7770f77c3485b` |
| `phase3r_B2048_r3_obs_per_second.csv` | 405 | `958ae3aedbbf761f733162fcb37813cc79b7417a8b2c7051a21c28a3a7d755db` |
| `phase3r_B2048_r3_run_meta.txt` | 260 | `da27bd53671e75579ed72770cf8b4071f6324112083b418ac8c8ff6a903aaefd` |
| `phase3r_B4096_r1_uart_capture.txt` | 26081 | `af415deb7ee4e26eeb9abd32db22db920cd3187533e372d0317e9e30c833ec37` |
| `phase3r_B4096_r1_sonar_timing.csv` | 10198 | `af5bacf0899d547712d97820f82de7d9c80e3ea81a369504e1c7421ea98c4162` |
| `phase3r_B4096_r1_wheel_timing.csv` | 12465 | `c4c8b0fab039117413221fdc4de525439854f561741f5437c546ae51242d435c` |
| `phase3r_B4096_r1_obs_per_second.csv` | 405 | `01f34216c5705dc15558c2f08fff59e060f814a18aed9699abe684414e60fe4b` |
| `phase3r_B4096_r1_run_meta.txt` | 260 | `f65878d17a10bd3c2fdc3188f60651b2967b5aaf1878d73d5d573856279ffdab` |
| `phase3r_B4096_r2_uart_capture.txt` | 26081 | `92338e7a307341eeafca5c17d7a7abe0cc396693bb8aa7a5b11d313d26189251` |
| `phase3r_B4096_r2_sonar_timing.csv` | 10198 | `cb1f7a68b378dd3e3e6e38c3cb3206e2402d54dd3d6c64ce1d71bacf8788928f` |
| `phase3r_B4096_r2_wheel_timing.csv` | 12465 | `8ce707810f6c73d6373fba5890e814fd5c3e5e6268dca5f1bba770bc6b8166e5` |
| `phase3r_B4096_r2_obs_per_second.csv` | 405 | `134bd9d7c2770bab22d7c6aaed92f84cc873e347825cacbad02288f33e36af39` |
| `phase3r_B4096_r2_run_meta.txt` | 260 | `f65878d17a10bd3c2fdc3188f60651b2967b5aaf1878d73d5d573856279ffdab` |
| `phase3r_B4096_r3_uart_capture.txt` | 26081 | `2cbe99c358469a2264c30e820654a8cce17fb725417399ab911284a2e507a080` |
| `phase3r_B4096_r3_sonar_timing.csv` | 10198 | `4cc4b106dc8da5ed4638ca21269c1dd69dc6b579dda00bf95c0c80e7a6aa2f8b` |
| `phase3r_B4096_r3_wheel_timing.csv` | 12465 | `cc2cb75b616654414786b153c8d49effea6965f14883ab8833a8e53110555c01` |
| `phase3r_B4096_r3_obs_per_second.csv` | 405 | `ea47a57ea1c0111185f6cec17202c36a06efec58f91998943863e8092f64cfba` |
| `phase3r_B4096_r3_run_meta.txt` | 260 | `f65878d17a10bd3c2fdc3188f60651b2967b5aaf1878d73d5d573856279ffdab` |
